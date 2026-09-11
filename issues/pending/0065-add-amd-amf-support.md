# AMD AMF のハードウェアエンコーダ/デコーダに対応する

- Created: 2026-09-11
- Completed: {YYYY-MM-DD}
- Branch: feature/add-amd-amf-support
- Polished: {YYYY-MM-DD}
- Reporter: @miosakuma

## 目的

Momo は NVIDIA Video Codec SDK / Intel VPL / Jetson / V4L2 / VideoToolbox のハードウェアエンコーダ・デコーダに対応しているが、AMD の GPU が持つ VCN (Video Core Next) には対応していない。デスクトップで AMD GPU を使う利用者が増えており、AMD AMF (Advanced Media Framework) を使った H.264 / H.265 / AV1 のハードウェアエンコード・デコードに対応する。

## 現状

- `src/video_codec_info.h` の `VideoCodecInfo::Type` に AMD はなく、`GetLinux` / `GetWindows` は AMD のエンジンを列挙しない
- `src/rtc/momo_video_encoder_factory.cpp` / `src/rtc/momo_video_decoder_factory.cpp` に AMD AMF の分岐がない
- `CMakeLists.txt` に AMD AMF を有効にするオプションがない
- Sora C++ SDK 側は AMD AMF に対応済みで、`sora::AMFContext` (`include/sora/amf_context.h`)、`sora::VideoCodecImplementation::kAmdAmf` (文字列は `amd_amf`)、`sora::GetAMFVideoCodecCapability` が利用できる。ただし Momo が現在取り込んでいる `src/sora-cpp-sdk/` には AMD AMF のファイルが含まれていない
- PR #380 で AMD AMF 対応を実装したが、Momo 独自の `src/hwenc_amf/` を持つ形だったため closed にした。Sora C++ SDK の仕組みに乗せる方針に変更した
- AMD AMF の動作には `amdgpu-install --usecase=graphics,amf --vulkan=pro` によるドライバー、Vulkan、`render` / `video` グループへの所属、self-hosted runner のセットアップが必要 (Sora C++ SDK 側で確認済み)
- Sora C++ SDK 側では AV1 デコードに課題が残っているとされている

## 設計方針

- Sora C++ SDK の AMD AMF の仕組み (`sora::AMFContext` / `sora::VideoCodecImplementation::kAmdAmf` / `sora::GetAMFVideoCodecCapability`) を利用し、Momo 独自の `src/hwenc_amf/` は持たない
- `VideoCodecInfo::Type` に `AMD` を追加し、`TypeToString` の名前を Sora C++ SDK の `amd_amf` に揃える (0063 の方針と整合させる)
- `CMakeLists.txt` に `USE_AMF_ENCODER` を追加し、`USE_NVCODEC_ENCODER` / `USE_VPL_ENCODER` と同様に `src/sora-cpp-sdk/` の AMD AMF 実装を取り込む
- `MomoVideoEncoderFactoryConfig` / `MomoVideoDecoderFactoryConfig` に AMF コンテキストを追加し、`momo_video_encoder_factory.cpp` / `momo_video_decoder_factory.cpp` に AMD の分岐を追加する
- `RTCManager` で AMF コンテキストを初期化し、`VideoCodecInfo::Get()` のエンジン列挙と `--video-codec-engines` の出力に AMD を追加する
- E2E テストで AMD AMF を使った H.264 / H.265 / AV1 の送受信を確認する

## 完了条件

- `--video-codec-engines` に AMD AMF (`AMD AMF [amd_amf]` など) が表示される
- `--h264-encoder amd_amf` などの指定で AMD AMF のハードウェアエンコーダ・デコーダが使われ、映像が送受信できる
- AMD AMF 環境の E2E テストが通る
- `CHANGES.md` に AMD AMF 対応が追記されている

## 解決方法

未着手 (方式が決まり PR 作成後に追記する)

## pending にした理由

AMD AMF の環境構築 (`amdgpu-install` によるドライバー、Vulkan、`render` / `video` グループ、self-hosted runner のセットアップ) が非常に大変で、不急のため。加えて PR #380 の実装を Sora C++ SDK の仕組みに乗せる方針へ変更しており、その設計を反映する必要があるため pending とする。
