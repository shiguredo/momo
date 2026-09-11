# Raspberry Pi で --hw-mjpeg-decoder を有効にすると FHD の H.264 映像が受信先で正しく受信されない

- Created: 2026-09-11
- Completed: {YYYY-MM-DD}
- Branch: feature/fix-raspberry-pi-hw-mjpeg-decoder-fhd
- Polished: {YYYY-MM-DD}
- Reporter: @torikizi

## 目的

Raspberry Pi 4 で `--hw-mjpeg-decoder` を有効にし、FHD の H.264 を配信すると、受信側の映像に緑の線が入る。リサイズ処理が正しく動いていないためで、FHD でも正しく映像が送受信できるようにする。

## 現状

- Raspberry Pi 4 / Bookworm で `./momo --no-audio-device --log-level 0 --hw-mjpeg-decoder true --resolution FHD test` を実行すると、受信側の映像に緑の線が入る
- HD では `--hw-mjpeg-decoder` を有効にしても正しく受信できる
- `--hw-mjpeg-decoder` を無効にすると問題は発生しない
- USB カメラでのみ発生し、Raspberry Pi Camera v3 を libcamera で使った場合は発生しない
- momo 2023.1.0 および 2025.1.0-canary.9 で再現する
- `--hw-mjpeg-decoder` は `MomoArgs::hw_mjpeg_decoder` から `V4L2VideoCapturerConfig::use_native` に渡り、`src/main.cpp` で `rtcm_config.hardware_encoder_only` にもなる
- native 時は `V4L2Capturer::OnCaptured` が MJPEG の `V4L2NativeBuffer` を生成し、`V4L2H264Encoder::Encode` が `V4L2DecodeConverter` (MJPEG デコード) と `V4L2ScaleConverter` (リサイズ) を経由して H.264 エンコードする (`src/sora-cpp-sdk/src/hwenc_v4l2/`)
- HD では発生せず FHD で発生するため、解像度に依存したリサイズ処理 (ストライド、クロップ、スケーラーへの入力サイズ) に原因がある可能性が高い
- 同じ事象は shiguredo/momo の公開 issue #354 にも記録されている

## 設計方針

- FHD で緑の線が出る箇所を特定する。`V4L2ScaleConverter` の入力 / 出力のサイズとストライド、`V4L2DecodeConverter` が返すフレームのサイズとストライドが一致しているかを確認する
- Raspberry Pi の V4L2 M2M スケーラーの制約 (アライメント、最大サイズ、対応 pixelformat) を確認し、FHD で必要な設定が抜けていないかを見る
- 原因箇所によっては sora-cpp-sdk 側の修正が必要になるため、momo の vendored コードと sora-cpp-sdk の両方を対象にする
- 修正後は Raspberry Pi 4 + USB カメラ + `--hw-mjpeg-decoder true` で FHD / HD の H.264 を配信し、受信側で緑の線が出ないことを確認する

## 完了条件

- Raspberry Pi 4 + USB カメラで `--hw-mjpeg-decoder true --resolution FHD` の H.264 を配信したとき、受信側の映像に緑の線が入らない
- HD など他の解像度でも緑の線が入らない
- `--hw-mjpeg-decoder false` の従来の動作が変わらない
- 修正が sora-cpp-sdk 側にも反映され、`LAST_UPDATED` が更新されている

## 解決方法

{YYYY-MM-DD} に追記する
