# hwenc_v4l2 を sora-cpp-sdk 側へ移す

- Created: 2025-09-24
- Completed: 2025-09-24
- Branch: feature/hwenc-v4l2-to-sora-cpp-sdk
- Polished: {YYYY-MM-DD}

## 目的

Raspberry Pi 向け V4L2 M2M エンコーダ / デコーダと関連キャプチャーを、momo 固有の `src/hwenc_v4l2/` から vendoring している `src/sora-cpp-sdk/` へ移し、Sora C++ SDK と同じ配置にする。

## 現状

`hwenc_v4l2` は momo 側に残っており、`CMakeLists.txt` と `src/rtc/momo_video_encoder_factory.cpp` / `momo_video_decoder_factory.cpp` が momo パスを直接参照していた。Sora C++ SDK へ移植済みの実装と二重管理になり、差分同期のコストが大きい。

## 設計方針

ソースとヘッダを `src/sora-cpp-sdk/src/hwenc_v4l2/` と `src/sora-cpp-sdk/include/sora/hwenc_v4l2/` に移す。momo 側は include パスと CMake の `target_sources` だけを SDK 側へ向ける。公開 CLI や実行時のエンコーダ選択は変えない。

## 完了条件

- `src/hwenc_v4l2/` がリポジトリから無くなっている
- `CMakeLists.txt` が `src/sora-cpp-sdk/src/hwenc_v4l2/` をコンパイルする
- エンコーダ / デコーダ factory が `sora/hwenc_v4l2/` を include する
- Raspberry Pi 向けビルドが通る

## 解決方法

PR #425 で以下を変更した。

- `libcamera_capturer` / `v4l2_capturer` / `v4l2_h264_encoder` / `v4l2_h264_decoder` などを `src/sora-cpp-sdk/` 配下へ移す
- `CMakeLists.txt` の `USE_V4L2_ENCODER` 分岐の `target_sources` を SDK パスに付け替える
- `src/rtc/momo_video_encoder_factory.cpp` は `sora/hwenc_v4l2/v4l2_h264_encoder.h` を、`src/rtc/momo_video_decoder_factory.cpp` は `sora/hwenc_v4l2/v4l2_h264_decoder.h` を include する
- `src/sora-cpp-sdk/diff-momo.sh` / `diff-sora-cpp-sdk.sh` の対象パスを更新する
