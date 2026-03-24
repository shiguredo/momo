# --version の詳細表示

## 概要

`--version` でパッケージ名とバージョン番号しか表示されない。

## 現状

- `src/main.rs` で `println!("{} {}", env!("CARGO_PKG_NAME"), env!("CARGO_PKG_VERSION"))` のみ
- 出力: `shiguredo_momo 2026.0.0`

## momo の出力例

```
WebRTC Native Client Momo 2024.1.0 (abc1234)
WebRTC: Shiguredo-Build M120 (6099.0 def5678)
Environment: [aarch64] Ubuntu 22.04.3 LTS (nvidia-l4t-core 35.4.1-20230801210015)
```

## 必要な実装

- コミットハッシュ: ビルド時に git describe 等で取得
- libwebrtc バージョン: shiguredo_webrtc からバージョン情報を取得
- 環境情報: OS 名/バージョン/アーキテクチャ
- ビルドフラグ: 有効な feature 一覧 (未実装、表示すべきフラグの設計判断が必要)

## 解決方法

- `build.rs` を追加し `git rev-parse --short HEAD` でコミットハッシュを `MOMO_COMMIT_SHORT` 環境変数に埋め込み
- `--version` 表示を 3 行に拡張:
  - `shiguredo_momo 2026.0.0 (abc1234)`
  - `WebRTC: Shiguredo-Build 0.146.0-canary.4`
  - `Environment: [aarch64] macOS 15.3`
- `metrics::build_environment_string()` を公開して共有

Completed: 2026-03-22

## Reopen: 2026-03-22

コミットハッシュ、libwebrtc バージョン、環境情報は実装済みだが、ビルドフラグ表示が未実装のまま close されていた。
momo は USE_JETSON_ENCODER / USE_NVCODEC_ENCODER / USE_V4L2_ENCODER / USE_VPL_ENCODER 等のコンパイルオプション一覧を表示しており、この機能が残作業として残っている。
