# sora-cpp-sdk を 2026.2.1 に同期し、SDL3 / CLI11 を最新化する

- Created: 2026-08-19
- Completed: {YYYY-MM-DD}
- Branch: feature/update-sora-cpp-sdk-sdl3-cli11
- Polished: {YYYY-MM-DD}

## 目的

正式リリースに向けて、vendoring している `src/sora-cpp-sdk/` を最新リリース `2026.2.1` (2026-08-18 公開) に同期し、`SDL3` を `3.4.14`、`CLI11` を `v2.7.2` に更新する。libwebrtc は m150 に固定し、m152 には上げない方針とする。

## 現状

- `DEPS` の依存バージョン: `SDL3_VERSION=3.2.24`、`CLI11_VERSION=v2.6.1`、`BOOST_VERSION=1.91.0`、`WEBRTC_BUILD_VERSION=m150.7871.3.0`
- `src/sora-cpp-sdk/LAST_UPDATED` は `LAST_UPDATED_SORA_CPP_SDK=bd0af595...` で、同期元の最新 `2026.2.1` から遅れている
- sora-cpp-sdk `2026.2.1` は Boost 1.92.0 を同梱しており、momo 側の Boost 1.91.0 との整合を検討する必要がある
- sora-cpp-sdk の同期スクリプトにバグがある
  - `src/sora-cpp-sdk/README.md` の手順 `./diff-sora-cpp-sdk.sh | patch -p1` は、スクリプトが引数 2 個必須のため実行不能
  - `copy-from-sora-cpp-sdk.sh` / `copy-to-sora-cpp-sdk.sh` は `third_party` をコピー対象に含めない (diff 側は含むのに非対称)

## 設計方針

1. `src/sora-cpp-sdk/` を同期元の `2026.2.1` に同期する (diff / copy スクリプトを利用)
2. 同期スクリプトの `third_party` 欠落と README 手順のバグを修正する
3. `DEPS` の `SDL3_VERSION` を `3.4.14` に、`CLI11_VERSION` を `v2.7.2` に更新する
4. Boost の 1.91.0 → 1.92.0 は sora-cpp-sdk 側との整合を確認して判断する
5. libwebrtc は m150 のまま変更しない
6. CI 側の依存バージョン二重管理を解消する
   - `e2e-test.yml` の `OPENH264_VERSION` が DEPS と別管理 (`v` 有無も不統一)
   - `build.yml` の `cuda_version` が DEPS (`12.9.1-1`) と別管理

## 完了条件

- `src/sora-cpp-sdk/LAST_UPDATED` が `2026.2.1` 相当のコミットハッシュに更新されている
- 全プラットフォーム (windows / macos / ubuntu / raspberry-pi-os / jetson) でビルドが通る
- E2E テストが通る
- 下記の vendored コードに対する重要指摘が、`2026.2.1` への同期で解消されているか確認されている。解消されていないものは別 issue を起票する
  - VPL の `VPL_CHECK_RESULT` が負の MFX_ERR_* を無視 (`vpl_utils.h`)
  - VPL の NV12 で `Data.UV` 未設定 (`vpl_video_encoder.cpp` / `vpl_video_decoder.cpp`)
  - Jetson / V4L2 の `configured_bitrate_bps_` 未初期化比較 (`jetson_video_encoder.cpp` / `v4l2_h264_encoder.cpp`)
  - IVF ヘッダ除去の size アンダーフロー (`jetson_video_encoder.cpp` / `nvcodec_video_encoder.cpp` / `vpl_video_encoder.cpp`)
  - Jetson デコーダの `CHUNK_SIZE` 超過 memcpy (`jetson_video_decoder.cpp`)
  - Jetson エンコーダの AV1 OBU サイズ 1 バイト固定読み (`jetson_video_encoder.cpp`)
  - NvCodec エンコーダの `frameRateNum=0` 除算ゼロ (`nvcodec_video_encoder.cpp`)
  - CUDA の `cuCtxPushCurrent` が例外パスで Pop されない (`nvcodec_decoder_cuda.cpp`)
  - V4L2Runner が POLLERR/HUP を無視し破棄ハング (`v4l2_runner.cpp`)
  - V4L2 系のデバイスパス直書き (`v4l2_converter.cpp`)
  - libcamerac の Span dangling (`libcamerac.cpp`)
  - libcamera の mmap `MAP_FAILED` 誤判定 (`libcamera_capturer.cpp`)
  - NvCodec エンコーダのフレームサイズ未検証 GPU コピー (`nvcodec_video_encoder.cpp`)
  - `v4l2_h264_decoder.cpp` の `Release()` 空実装

## 解決方法

未着手 (PR 作成後に追記する)
