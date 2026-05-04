# プロキシ対応の実装

## 概要

プロキシ関連の CLI オプションがパースされるが未使用。

## 現状

- `src/main.rs` で `_proxy_url`, `_proxy_username`, `_proxy_password` に束縛されるだけ
- MomoConfig に含まれず、WebSocket 接続処理に渡されていない

## 対応方針

- Sora モードのみ対応。sora_sdk の `ProxyInfo` API を使用する
- P2P / Ayame モードは非対応

## 必要な実装

- `--proxy-url`: sora_sdk の `ProxyInfo::url` に設定
- `--proxy-username` / `--proxy-password`: sora_sdk の `ProxyInfo` に設定
- パース済みの値を `SoraClientBuilder::proxy()` に渡す
