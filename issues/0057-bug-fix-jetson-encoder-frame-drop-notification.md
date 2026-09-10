# Jetson エンコーダが OnFrameDropped 通知と is_end_of_temporal_unit 設定に対応していない

- Created: 2026-09-10
- Completed: {YYYY-MM-DD}
- Branch: feature/fix-jetson-encoder-frame-drop-notification
- Polished: 2026-09-10

## 目的

libwebrtc は VideoEncoder に対して、フレームをドロップしたときは `EncodedImageCallback::OnFrameDropped()` で通知し、送出する `EncodedImage` に `is_end_of_temporal_unit` を設定することを求めている。この要件は Momo が利用する m150 に含まれている。

Sora C++ SDK 側では独自エンコーダーの対応が完了しており、Momo も sora-cpp-sdk 2026.2.1 への同期 (`26d5bef7`) で OpenH264 / NvCodec / VPL / V4L2 の対応を取り込み済みである。しかし、sora-cpp-sdk の develop には存在せず Momo 側だけで保守している Jetson エンコーダー (`JetsonVideoEncoder`) だけが対応から漏れている。Jetson でも同じ要求を満たすように修正する。

参考:

- https://issues.webrtc.org/issues/467444018
- https://source.chromium.org/chromium/_/webrtc/src/+/54ff9c19789b36a18d5ad9576be3775255caa279

## 現状

- `src/sora-cpp-sdk/src/hwenc_jetson/jetson_video_encoder.cpp` の `JetsonVideoEncoder::SendFrame()` は `OnEncodedImage()` を呼ぶ前に `encoded_image_.set_end_of_temporal_unit()` を呼んでいない
  - Momo 内の他のエンコーダー (`OpenH264VideoEncoder` / `NvCodecVideoEncoderImpl` / `VplVideoEncoderImpl` / `V4L2H264Encoder`) はすべて設定済み
- `JetsonVideoEncoder::EncodeFinishedCallback()` は以下の経路で出力フレームを送信せずにスキップしているが、`OnFrameDropped()` を呼んでいない
  - `frame_params_` が空で、出力に対応する `FrameParams` が見つからない
  - キュー先頭の `FrameParams::timestamp_us` が出力の timestamp と一致しない (`timestamp_us` が出力より古い `FrameParams` は出力が得られないままキューから破棄される)
  - `getMetadata()` が失敗する

### この状態による影響

- `is_end_of_temporal_unit` が未設定のため、WebRTC がテンポラルユニットの完了を判定できず、古いフレーム情報の解放が遅れる可能性がある。将来 libwebrtc がフラグ未設定をエラーとして厳格化する可能性がある
- ドロップしたフレームが `OnFrameDropped()` で通知されないため、Jetson エンコーダーのフレームドロップが libwebrtc のフレームドロップ統計や品質適応に反映されない

## 設計方針

- `JetsonVideoEncoder::SendFrame()` で `OnEncodedImage()` の直前に `encoded_image_.set_end_of_temporal_unit(true)` を設定する
  - Jetson エンコーダーは H.264 / H.265 / VP8 / VP9 / AV1 のいずれも単一空間レイヤーで運用しており、1 入力 = 1 出力 = 1 テンポラルユニットなので `true` で正しい。NvCodec / VPL / V4L2 と同じ扱いにする
  - 単一ストリーム (SimulcastEncoderAdapter の bypass) ではエンコーダーが設定した値がそのまま使われるため、エンコーダー自身で設定する必要がある
- `JetsonVideoEncoder::EncodeFinishedCallback()` で、ドロップしたフレームを `FrameParams::timestamp_rtp` から特定できる場合は `OnFrameDropped(rtp_timestamp, 0, true)` を呼ぶ
  - do-while ループで出力 timestamp より古い `FrameParams` を pop した場合、それらは出力が得られなかったフレームなので、pop したそれぞれについて通知する。pop 直後にキューが空になっても、pop 済みのフレームは通知する
  - `getMetadata()` 失敗時は、一致した `FrameParams` からフレームを特定できるため通知する
  - 出力 timestamp より古い `FrameParams` を 1 件も pop しておらず、出力に対応する `FrameParams` も存在しない場合だけ、フレームを特定できないため通知しない
  - ループを抜けた後に `params->timestamp_us` が出力 timestamp より大きくなる分岐 (出力より新しい `FrameParams` を pop した場合) は本 issue の通知対象外とし、現状の破棄挙動のままとする
- 参照実装は libwebrtc の `h264_encoder_impl` と `libaom_av1_encoder`、および Sora C++ SDK の HWA エンコーダーの対応とする

## 完了条件

- `JetsonVideoEncoder` が送出する各フレームの `EncodedImage` に `is_end_of_temporal_unit` が設定されること
- `JetsonVideoEncoder` がドロップしたフレームを `OnFrameDropped()` で通知できること (E2E で決定的に再現できないためコードレビューで確認する)
- CI の `ubuntu-22.04_armv8_jetson` ビルドが通ること
- Jetson 実機で H.264 の映像送信が正常に行われること
- `python3 run.py format` で clang-format の差分が出ないこと
- `CHANGES.md` の `## develop` に `[FIX]` を追記すること

## 解決方法

未着手 (PR 作成後に追記する)
