# --force-i420 / --force-yuy2 / --force-nv12 の実装

## 概要

キャプチャフォーマットの強制指定オプションが存在しない。

## 現状

- CLI オプション自体が未定義
- momo-rs は NV12/YUY2 を自動検出して I420 に変換している

## 必要な実装

- `--force-i420`, `--force-yuy2`, `--force-nv12` CLI オプション追加 (互いに排他的)
- 映像キャプチャ時に指定フォーマットを強制する

## 解決方法

- `ForcePixelFormat` 列挙型を `src/main.rs` に定義し、CLI フラグ 3 つ (排他チェック付き) を追加
- `CommonConfig` → 各モード Config → `VideoCaptureConfig.pixel_format` に伝搬
- shiguredo_video_device の `VideoCaptureConfig.pixel_format: Option<PixelFormat>` に `Some(...)` を渡すことで強制

Completed: 2026-03-22
