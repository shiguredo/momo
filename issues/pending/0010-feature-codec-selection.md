# コーデックエンコーダ/デコーダ選択オプションの実装

## 概要

コーデックのエンコーダ/デコーダ選択オプションがパースされるが未使用。

## 現状

- `src/main.rs` で以下が `_` 付き変数に束縛されるだけ:
  - `_vp8_encoder` / `_vp8_decoder`
  - `_vp9_encoder` / `_vp9_decoder`
  - `_av1_encoder` / `_av1_decoder`
  - `_h264_encoder` / `_h264_decoder`
  - `_h265_encoder` / `_h265_decoder`
  - `_video_codec_engines`
  - `_openh264`
  - `_hw_mjpeg_decoder`

## 必要な実装

- `--{codec}-encoder` / `--{codec}-decoder`: エンコーダ/デコーダバックエンドの選択
  - momo の選択肢: default / software / jetson / nvidia / vpl / videotoolbox / v4l2
- `--video-codec-engines`: 利用可能なエンコーダ/デコーダの一覧表示
- `--openh264`: OpenH264 動的ライブラリパスの指定
- `--hw-mjpeg-decoder`: MJPEG ハードウェアデコーダの有効化

## pending 理由

V4L2 以外の HW バックエンド (Jetson #0011 / NVIDIA・Intel・VideoToolbox #0012) が未実装のため、
現状ではソフトウェアエンコーダしか選択肢がなく `default` = `software` となり選択する意味がない。
HW バックエンドの実装が完了してから対応する。
