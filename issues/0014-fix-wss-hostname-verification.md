# WSS 接続でサーバ証明書のホスト名検証が行われていない

- Created: 2026-08-19
- Completed: {YYYY-MM-DD}
- Branch: feature/fix-websocket-hostname-verification
- Polished: {YYYY-MM-DD}

## 目的

Momo が Sora / Ayame / P2P のシグナリングサーバへ WSS 接続する際、提示された証明書の**チェーン検証のみ**を行っており、**ホスト名 (SNI) との一致検証が一切行われていない**。このため正規 CA が発行した別ドメインの有効な証明書を提示する MITM 攻撃に対して脆弱であり、`--insecure` を指定しなくても中間者攻撃が成立する。これを修正する。

## 現状

- `src/websocket.cpp` の `Websocket::InitWss()` 内の verify コールバックは、`preverified == false` のとき `SSLVerifier::VerifyX509()` を呼ぶ
- `src/ssl_verifier.cpp` の `SSLVerifier::VerifyX509()` は `X509_verify_cert()` によるチェーン / 有効期限の検証のみで、`X509_check_host` / `SSL_set1_host` / `X509_VERIFY_PARAM_set1_host` 相当のホスト名照合がない
- `src/` 配下全体を検索してもホスト名照合のコードは存在しない

## 設計方針

- `Websocket` が保持する接続先ホスト名 (URL パース済みの `parts_.host`) を元に、verify コールバック内で証明書の SAN / CN を検証する
- `X509_check_host` を利用し、ワイルドカード証明書の扱いは OpenSSL 標準の挙動に従う
- `--insecure` 指定時は引き続き検証をスキップする
- 検証に失敗した場合は接続を拒否し、エラーを呼び出し元の `on_connect` へ通知する

## 完了条件

- 別ドメインの有効な証明書を提示された場合に接続が拒否される
- 正しいホスト名の証明書で接続が成功する
- `--insecure` 指定時は従来通り接続が成功する
- Sora / Ayame / P2P の各モードの E2E テストが通る

## 解決方法

未着手 (PR 作成後に追記する)
