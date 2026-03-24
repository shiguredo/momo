# H.264 HW エンコード (Jetson) の実装

## 概要

NVIDIA Jetson ハードウェアエンコーダー (H.264/H.265) が未実装。

## 現状

- momo は USE_JETSON_ENCODER コンパイルオプションで対応
- momo-rs には該当実装なし

## 必要な実装

- Jetson ハードウェアエンコーダーの WebRTC 統合
- VideoEncoderFactory への組み込み
