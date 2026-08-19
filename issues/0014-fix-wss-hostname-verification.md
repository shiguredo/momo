# WSS 接続でサーバ証明書のホスト名検証が行われていない

- Created: 2026-08-19
- Completed: {YYYY-MM-DD}
- Branch: feature/fix-websocket-hostname-verification
- Polished: 2026-08-19
- Milestone: 2026.1.0

## 目的

Momo が Sora / Ayame / P2P のシグナリングサーバへ WSS 接続する際、提示された証明書の**チェーン検証のみ**を行っており、**ホスト名 (SNI) との一致検証が一切行われていない**。このため正規 CA が発行した別ドメインの有効な証明書を提示する MITM 攻撃に対して脆弱であり、`--insecure` を指定しなくても中間者攻撃が成立する。これを修正する。

## 現状

- `src/websocket.cpp` の `Websocket::InitWss()` 内の verify コールバックは、`preverified == true` なら即 `true` を返し、`preverified == false` のときのみ `SSLVerifier::VerifyX509()` を呼ぶ。`InitWss` は `Websocket` コンストラクタと `Websocket::OnReadProxy()` から呼ばれ、コールバックは `insecure` フラグのみを capture している
- `src/ssl_verifier.cpp` の `SSLVerifier::VerifyX509()` は `X509_STORE` を自前構築して `X509_verify_cert()` によるチェーン / 有効期限の検証のみを行い、`X509_check_host` / `X509_check_ip` / `SSL_set1_host` / `X509_VERIFY_PARAM_set1_host` 相当のホスト名照合がない
- `src/` 配下全体を検索してもホスト名照合のコードは存在しない。`Websocket::Connect()` / `Websocket::ConnectProxy()` で `SSL_set_tlsext_host_name()` による SNI 設定は行っているが、証明書の SAN / CN と `parts_.host` の一致検証はしていない。`RTCSSLVerifier` (`src/rtc/rtc_ssl_verifier.cpp` の `VerifyChain()`) も同 `VerifyX509()` を呼ぶが、こちらは DTLS / TURN-TLS 経路でホスト名の概念がない
- `Websocket::Connect()` で `URLParts::Parse()` 後に `parts_.host` が確定するが、`InitWss()` はコンストラクタ時点でも呼ばれるため、verify コールバック内で `parts_.host` を参照する場合は capture タイミングと `this` の寿命に注意が必要。`src/url_parts.h` の `URLParts::Parse()` は `:` で単純分割しており、IPv6 リテラル `[2001:db8::1]:443` を正しく分解できない

## 設計方針

- `preverified` 分岐は削除し、`preverified` の値に関わらずホスト名検証を行う。コールバックの先頭で `if (this->insecure_) return true;` のみを残し、それ以外は `SSLVerifier::VerifyX509(cert, chain, host)` の結果を返す。`preverified == true` でも別ドメインの有効証明書で `true` を返す脆弱性を防ぐため、チェーン検証とホスト名一致の両方が成功した場合のみ `true` を返す。OpenSSL 前提で `X509_check_host` / `X509_check_ip` が利用可能であることを確認済みとする
- `SSLVerifier::VerifyX509()` はオーバーロードでホスト名対応を追加する (`VerifyX509(X509*, STACK_OF(X509)*, const std::string& host)` を新設し、既存の `VerifyX509(X509*, STACK_OF(X509)*)` は `host = ""` で新関数へ委譲して互換性を保つ)。ホスト名が空の場合はチェーン検証のみを行い、`src/rtc/rtc_ssl_verifier.cpp` の DTLS / TURN-TLS 経路は既存の 2 引数版を呼び出して互換性を保つ。`src/ssl_verifier.h` の宣言と `src/rtc/rtc_ssl_verifier.cpp` の呼び出しは変更不要とする
- ホスト名検証はチェーン検証 (`X509_verify_cert()`) 成功後に行う。正規化は `Websocket` 側で一括して行い、順序は `1. IPv6 ブラケット `[` `]` 除去 2. 末尾ドット除去 3. `inet_pton()` で IP 判定` とする。正規化後の文字列が IP なら `X509_check_ip()`、DNS 名なら `X509_check_host()` で検証する。フラグは `0` (OpenSSL デフォルト、RFC 6125 Section 6.4.3 に準じて左端ラベルのみのワイルドカードを許可、SAN が無い場合の CN フォールバックを含む) とし、戻り値 `1` が一致、`0` が不一致、`-1` が内部エラーとして不一致と同等に失敗させる。ワイルドカードで `*.example.com` が `foo.example.com` に一致し `foo.bar.example.com` に不一致となる挙動は OpenSSL 標準に従う
- `Websocket` 側は `parts_.host` を verify コールバックが参照できるように、`InitWss()` の capture を `[this]` に変更し `this->parts_.host` と `this->insecure_` を参照する。`insecure` の値 capture は廃止して二重管理を解消する。`Connect()` と `OnReadProxy()` で `parts_.host` 確定後に再設定する必要はなく、`this` 経由で最新ホストを参照する。`wss_` が `OnReadProxy()` で再生成される点と `strand` 上での寿命に注意し、`Websocket` の寿命が `wss_` より長いことを前提とする
- `--insecure` 指定時は検証をスキップする (コールバック先頭で `if (this->insecure_) return true;`)
- ホスト名検証に失敗した場合は `X509_STORE_CTX_set_error(ctx.native_handle(), X509_V_ERR_APPLICATION_VERIFICATION)` を設定した上で `false` を返し、ハンドシェイクを失敗させる。`X509_check_host` / `X509_check_ip` の不一致は `ERR_get_error()` に積まれないため、`ERR_get_error()` ではなく `X509_V_ERR_APPLICATION_VERIFICATION` を明示的に設定する。`Websocket::OnSSLHandshake()` 経由で `on_connect` にエラーが通知される。エラーログは `SSLVerifier::VerifyX509()` 内で英語で出力し、ホスト名と `X509_check_host` / `X509_check_ip` の戻り値を含める
- プロキシ経由 (`Websocket::OnReadProxy()`) でも同様に `parts_.host` (接続先ホスト) で検証する。プロキシ自体のホスト名ではなく、SNI も `parts_.host` であるためエンドツーエンドの検証となる
- `URLParts::Parse()` の IPv6 対応は本 issue ではスコープ外とする
- 本 issue のスコープは WSS (Sora / Ayame / P2P) のみとし、TURN-TLS / DTLS の `RTCSSLVerifier` はホスト名検証対象外であることを明記する。`0024-fix-expired-r3-certificate` と同一ファイルを同時変更するため、`0024` を先にマージしてから本 issue を rebase する順序を推奨する

## 完了条件

- 別ドメインの有効な証明書を提示された場合に WSS ハンドシェイクが失敗し、`on_connect` にエラーが通知される。ローカルで `openssl s_server` による MITM 再現または `wrong.host.badssl.com` 等で手動検証する
- 正しいホスト名の証明書で WSS 接続が成功する。ワイルドカード証明書 (`*.example.com` が `foo.example.com` に一致) も含めて検証する
- `--insecure` 指定時はホスト名不一致でも接続が成功する
- Sora / Ayame / P2P の各モードの E2E テストが通る。ネガティブケース (別ドメイン証明書での失敗) は手動検証で補完する

## 解決方法

未着手 (PR 作成後に追記する)
