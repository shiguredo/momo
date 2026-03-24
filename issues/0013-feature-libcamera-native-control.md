# libcamera の --use-libcamera-native / --libcamera-control の実装

## 概要

libcamera のネイティブバッファ出力とコントロール設定が未実装。

## 現状

- `--use-libcamera` は実装済み (`src/libcamera.rs`)
- `--use-libcamera-native` と `--libcamera-control` は CLI オプション自体が存在しない
- 現状は DMA-BUF を mmap して読み取り、I420Buffer に変換している

## 必要な実装

- `--use-libcamera-native`: DMA-BUF ネイティブバッファをエンコーダに直接渡す (ゼロコピー)
  - I420Buffer 変換をスキップしてメモリコピーを削減
- `--libcamera-control KEY VALUE`: libcamera のコントロールパラメータ設定
  - 露出、ホワイトバランス、明るさ等
