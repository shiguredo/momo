# クライアント証明書認証の実装

## 概要

クライアント証明書関連の CLI オプションがパースされるが未使用。

## 現状

- `src/main.rs` で `_client_cert`, `_client_key` に束縛されるだけ
- CommonConfig に含まれず、TLS 接続処理に渡されていない
- 証明書ファイルの読み込みコードも存在しない

## 必要な実装

- `--client-cert`: PEM 形式クライアント証明書ファイルの読み込み
- `--client-key`: PEM 形式秘密鍵ファイルの読み込み
- rustls の TLS 設定にクライアント証明書を組み込む
- `--insecure` は #0021 として独立 issue に分離済み

## 解決方法

- `_client_cert` / `_client_key` のアンダースコアを外して有効化
- `--client-cert` / `--client-key` の排他バリデーション (両方必須) を `main.rs` に追加
- PEM ファイルを読み込み `(cert_pem, key_pem)` タプルとして `CommonConfig` に格納
- Ayame モード: `rustls_pemfile` で PEM をパースし、`rustls::ClientConfig` に `with_client_auth_cert()` または `SingleCertResolver` で設定
- Sora モード: `sora_sdk::SoraClientBuilder::client_cert(cert_pem, key_pem)` で設定
- `rustls-pemfile` v2 を依存に追加 (ayame feature で有効化)

Completed: 2026-03-22
