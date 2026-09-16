# WSS 接続でサーバ証明書のホスト名検証が行われていない

- Created: 2026-08-19
- Completed: 2026-08-21
- Branch: feature/fix-websocket-hostname-verification
- Polished: 2026-08-20
- Milestone: 2026.1.0

## 目的

Momo が Sora / Ayame のシグナリングサーバへ WSS 接続する際、提示された証明書の**チェーン検証のみ**を行っており、**証明書の SAN / CN と接続先ホスト名 (SNI に設定する値) の一致検証が一切行われていない**。このため正規 CA が発行した別ドメインの有効な証明書を提示する MITM 攻撃に対して脆弱であり、`--insecure` を指定しなくても中間者攻撃が成立する。これを修正する。

## 現状

- `src/websocket.cpp` の `Websocket::InitWss()` 内の verify コールバックは、`preverified == true` なら即 `true` を返し、`preverified == false` のときのみ `SSLVerifier::VerifyX509()` を呼ぶ。`InitWss` は ssl_tag コンストラクタと `Websocket::OnReadProxy()` から呼ばれ、コールバックは `insecure` フラグのみを capture している
- `src/ssl_verifier.cpp` の `SSLVerifier::VerifyX509()` は `X509_STORE` を自前構築して `X509_verify_cert()` によるチェーン / 有効期限の検証のみを行い、`X509_check_host` / `X509_check_ip` / `SSL_set1_host` / `X509_VERIFY_PARAM_set1_host` 相当のホスト名照合がない
- `src/` 配下全体を検索してもホスト名照合のコードは存在しない。`Websocket::Connect()` とプロキシ経路の `Websocket::OnReadProxy()` で `SSL_set_tlsext_host_name()` による SNI 設定は行っているが、証明書の SAN / CN と `parts_.host` の一致検証はしていない。`RTCSSLVerifier` (`src/rtc/rtc_ssl_verifier.cpp` の `VerifyChain()`) も同 `VerifyX509()` を呼ぶが、こちらは DTLS / TURN-TLS 経路でホスト名の概念がない
- `Websocket::Connect()` で `URLParts::Parse()` 後に `parts_.host` が確定するが、`InitWss()` はコンストラクタ時点でも呼ばれるため、verify コールバック内で `parts_.host` を参照する場合は capture タイミングと `this` の寿命に注意が必要。`src/url_parts.h` の `URLParts::Parse()` は `:` で単純分割しており、IPv6 リテラル `[2001:db8::1]:443` を正しく分解できない
- `Websocket::Websocket(https_proxy_tag, ...)` は `insecure` パラメータを `insecure_` メンバに保存しておらず、プロキシ経由では `--insecure` が効かない状態になっている。本 issue の `[this]` capture 化に伴い `insecure_(insecure)` の保存を追加する (設計方針 4)
- 本 issue の検証コードの実行対象は libwebrtc が同梱する BoringSSL (`CMakeLists.txt` で `OPENSSL_IS_BORINGSSL` が定義され `WebRTC::WebRTC` をリンク) である。`X509_check_host` / `X509_check_ip` / `X509_V_ERR_APPLICATION_VERIFICATION` は BoringSSL にも存在し利用可能

## 設計方針

1. verify コールバックはチェーンを構成する各証明書 (リーフ / 中間) に対して呼ばれるため、ホスト名検証はリーフ証明書 (`X509_STORE_CTX_get_error_depth(ctx) == 0`) に対してのみ行う。コールバックの先頭で `if (this->insecure_) return true;` を残し (--insecure 指定時は検証をスキップ)、リーフ証明書では `SSLVerifier::VerifyX509(cert, chain, host)` の結果を返す (チェーン検証とホスト名一致の両方に成功した場合のみ `true`)。リーフ以外の証明書では `true` を返す (リーフの `VerifyX509` が `X509_verify_cert()` でチェーン全体を検証済みのため。`preverified` は使用しない)。これにより別ドメインの有効証明書で `true` を返す脆弱性を防ぐ。libwebrtc が同梱する BoringSSL の `X509_check_host` / `X509_check_ip` が利用可能であることを確認済みとする (現状参照)
2. `SSLVerifier::VerifyX509()` はオーバーロードでホスト名対応を追加する (3 引数版 `VerifyX509(X509*, STACK_OF(X509)*, const std::string& host)` を新設し、チェーン検証失敗とホスト名不一致を区別する戻り値 (enum 等) を返す。`src/ssl_verifier.h` に 3 引数版の宣言を追加する)。既存の 2 引数版 `VerifyX509(X509*, STACK_OF(X509)*)` は公開戻り値を `bool` のまま維持し、`host = ""` で 3 引数版へ委譲して enum → bool に変換して互換性を保つ (rtc_ssl_verifier.cpp の呼び出しは変更不要)。ホスト名が空の場合はチェーン検証のみを行う
3. ホスト名検証はチェーン検証 (`X509_verify_cert()`) 成功後に行う。ホスト名が空の場合は設計方針 2 のとおりチェーン検証のみ行い、ホスト名照合は行わない。正規化のうち末尾ドット除去は `Websocket` 側でローカルコピーに対して行い (`parts_.host` 自体は SNI と Host ヘッダに使うため変更しない)、IP 判定と照合は `VerifyX509` 内で行う。順序は `1. 末尾ドット除去 2. inet_pton()` で IP 判定 (AF_INET → AF_INET6 の順で試す) とする。IPv6 リテラルは `URLParts::Parse()` が分解できないため対象外 (設計方針 7)。正規化後の文字列が IP (IPv4 リテラル等) なら `X509_check_ip(x509, ip_bytes, length, 0)` (inet_pton の出力バイト列と長さを渡す)、DNS 名なら `X509_check_host(x509, host.c_str(), host.size(), 0, nullptr)` で検証する。フラグは `0` (BoringSSL デフォルト。左端ラベルのみのワイルドカードを許可し、SAN 拡張が存在すれば SAN のみ、存在しなければ CN にフォールバックする)。戻り値 `1` が一致、`0` が不一致、負数が内部エラーとして不一致と同等に失敗させる。ワイルドカードで `*.example.com` が `foo.example.com` に一致し `foo.bar.example.com` に不一致となる挙動は標準に従う
4. `Websocket` 側は `parts_.host` を verify コールバックが参照できるように、`InitWss()` の capture を `[this]` に変更し `this->parts_.host` と `this->insecure_` を参照する。`insecure` の値 capture は廃止して二重管理を解消し、`InitWss()` の `bool insecure` パラメータも廃止する (呼び出し側は `InitWss(wss_.get())` になる)。`Websocket::Websocket(https_proxy_tag, ...)` に `insecure_(insecure)` の保存を追加し、プロキシ経由でも `--insecure` が有効になるようにする。`Connect()` と `OnReadProxy()` で `parts_.host` 確定後に再設定する必要はなく、`this` 経由で最新ホストを参照する。`wss_` が `OnReadProxy()` で再生成される点と `strand` 上での寿命に注意し、`Websocket` の寿命が `wss_` より長いことを前提とする
5. ホスト名検証に失敗した場合は、verify コールバックが `X509_STORE_CTX_set_error(ctx.native_handle(), X509_V_ERR_APPLICATION_VERIFICATION)` を設定した上で `false` を返し、ハンドシェイクを失敗させる。`VerifyX509()` はコールバックの `ctx` に到達できないため、チェーン検証失敗とホスト名不一致を区別できる戻り値 (enum 等) を返し、ホスト名不一致の場合のみコールバックがエラーを設定する。`X509_check_host` / `X509_check_ip` の不一致は `ERR_get_error()` に積まれないため、`ERR_get_error()` ではなく `X509_V_ERR_APPLICATION_VERIFICATION` を明示的に設定する。`Websocket::OnSSLHandshake()` 経由で `on_connect` にエラーが通知される。エラーログは `SSLVerifier::VerifyX509()` 内で英語で出力し、ホスト名と `X509_check_host` / `X509_check_ip` の戻り値を含める
6. プロキシ経由 (`Websocket::OnReadProxy()`) でも同様に `parts_.host` (接続先ホスト) で検証する。プロキシ自体のホスト名ではなく、SNI も `parts_.host` であるためエンドツーエンドの検証となる
7. `URLParts::Parse()` の IPv6 対応は本 issue ではスコープ外とする
8. 本 issue のスコープは WSS (Sora / Ayame) のみとし、TURN-TLS / DTLS の `RTCSSLVerifier` はホスト名検証対象外であることを明記する。P2P モードは WS (非 TLS) のサーバであり WSS クライアント接続を持たないため対象外とする。`0024-fix-expired-r3-certificate` と同一ファイル (`src/ssl_verifier.cpp`) を同時変更するため、`0024` を先にマージしてから本 issue を rebase する順序を推奨する。`0025-fix-proxy-connect-status` も `src/websocket.cpp` の `OnReadProxy()` を変更するため、併せてマージ順序を調整する

## 完了条件

- 別ドメインの有効な証明書を提示された場合に WSS ハンドシェイクが失敗し、`on_connect` にエラーが通知される。ローカルで `openssl s_server` による MITM 再現または `wrong.host.badssl.com` 等で手動検証する
- 正しいホスト名の証明書で WSS 接続が成功する。ワイルドカード証明書 (`*.example.com` が `foo.example.com` に一致) も含めて検証する
- `--insecure` 指定時はホスト名不一致でも接続が成功する (プロキシ経由を含む)
- Sora / Ayame の各モードの E2E テストが通る。ネガティブケース (別ドメイン証明書での失敗) は手動検証で補完する。P2P モードは WSS (TLS) を使用しないため検証対象外とする

## 解決方法

- `SSLVerifier::VerifyX509` にホスト名引数付きオーバーロードを追加し、チェーン検証成功後に `X509_check_host` / `X509_check_ip` で照合するようにした
- `Websocket::InitWss` の verify コールバックから `preverified` 分岐を削除し、`[this]` 経由で `parts_.host` と `insecure_` を参照するようにした
- ホスト名は Websocket 側で正規化 (IPv6 ブラケット除去・末尾ドット除去) してから渡す
- 検証失敗時は `X509_V_ERR_APPLICATION_VERIFICATION` を設定してハンドシェイクを失敗させる
- HTTP プロキシ経路でも接続先ホストで検証し、`https_proxy` コンストラクタの `insecure_` 初期化漏れを修正した
- CI の全プラットフォームビルドおよび WSS を含む E2E が通過したことを確認した
- PR の CI 成果物バイナリで手動確認した
  - `wrong.host.badssl.com` 指定時: momo から Handshake Failure の Alert が送られる
  - `--insecure` 追加時: Alert は送られず Application Data のやり取りまで進む
- PR: https://github.com/shiguredo/momo/pull/463
