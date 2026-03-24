# メトリクスレスポンスの environment を詳細化する

## 概要

momo-rs の environment は `"OS/ARCH"` 形式だが、momo はより詳細な情報を返す。

## 現状

- `src/metrics.rs` で `format!("{}/{}", std::env::consts::OS, std::env::consts::ARCH)` のみ
- momo は `"[ARCH] OS_DETAIL"` 形式:
  - Windows: `[x64] Windows 10.0 Build 19045`
  - macOS: `[arm64] macOS 14.2`
  - Linux: `[aarch64] Ubuntu 22.04.3 LTS` (/etc/os-release の PRETTY_NAME)
  - Jetson: 上記 + `(nvidia-l4t-core バージョン)`

## 必要な実装

- OS 詳細情報の取得 (Linux: /etc/os-release、macOS: sw_vers 等)
- `"[ARCH] OS_DETAIL"` 形式でのフォーマット

## 解決方法

- `build_environment_string()` 関数を追加し `"[ARCH] OS_DETAIL"` 形式で環境情報を構築
- macOS: `sw_vers` コマンドから ProductName + ProductVersion を取得
- Linux: `/etc/os-release` の PRETTY_NAME を取得
- その他 OS: `std::env::consts::OS` をフォールバックとして使用

Completed: 2026-03-22
