# `--insecure` オプションの実装

Created: 2026-03-22
Model: Opus 4.6

## 概要

`--insecure` オプションがパースされるが未使用。SSL 証明書検証をスキップする機能が動作しない。

## 根拠

momo では開発環境やオレオレ証明書を使うサーバーへの接続に `--insecure` が必須。
現在の momo-rs は証明書検証を常に行うため、自己署名証明書のサーバーに接続できない。

## モード別の実装方針

### Sora モード

sora_sdk の `SoraClientBuilder` に API あり:
- `.insecure(true)` → WebSocket シグナリングの TLS 証明書検証スキップ
- `.turn_tls_insecure(true)` → TURN-TLS の証明書検証スキップ

### Ayame モード

`src/ayame/signaling.rs` で `rustls::ClientConfig::with_platform_verifier()` を使用。
証明書検証をバイパスするには rustls の `dangerous_configuration` で `ServerCertVerifier` をカスタム実装する必要がある。

### P2P モード

P2P モードは TLS を使用しないため対象外。

## 解決方法

- Ayame モード: rustls の `dangerous()` API で `NoVerifier` (全証明書を受け入れるダミー検証器) を設定
- Sora モード: sora_sdk の `.insecure(true)` で WSS 証明書検証スキップ + `.turn_tls_insecure(true)` で TURN-TLS 証明書検証スキップ
- P2P モードは TLS を使用しないため対象外

Completed: 2026-03-22
