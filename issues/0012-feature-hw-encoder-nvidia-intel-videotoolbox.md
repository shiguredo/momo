# HW エンコード (NVIDIA / Intel / VideoToolbox) の実装

## 概要

NVIDIA (NvCodec)、Intel (oneVPL)、Apple VideoToolbox のハードウェアエンコーダーが未実装。

## 現状

- momo は以下に対応:
  - NVIDIA: USE_NVCODEC_ENCODER (CUDA 対応 GPU、H.264/H.265)
  - Intel: USE_VPL_ENCODER (oneVPL、H.264/H.265)
  - VideoToolbox: macOS (H.264/H.265)
- momo-rs には該当実装なし (V4L2 のみ実装済み)

## 必要な実装

- NVIDIA NvCodec エンコーダーの WebRTC 統合
- Intel oneVPL エンコーダーの WebRTC 統合
- Apple VideoToolbox エンコーダーの WebRTC 統合
- 各 VideoEncoderFactory への組み込み
