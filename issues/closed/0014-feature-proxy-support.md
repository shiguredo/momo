# プロキシ対応の実装

Completed: 2026-05-04

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

## 解決方法

- `src/main.rs` で `_proxy_url` / `_proxy_username` / `_proxy_password` を `proxy_url` / `proxy_username` / `proxy_password` にリネーム (アンダースコアを外す)
- `MomoConfig` に `proxy_url` / `proxy_username` / `proxy_password` フィールドを追加し、各モードに伝搬可能にする
- `sora::SoraConfig` に同フィールドを追加
- `src/sora/mod.rs` で `ProxyInfo` を import し、`proxy_url` が指定された場合のみ `SoraConnectionBuilder::proxy()` に渡す
- P2P / Ayame モードはオプションを受け取るが利用しない (CLI のヘルプにも `(Sora mode only)` を明記)
