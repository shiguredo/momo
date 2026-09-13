# NvCodec エンコーダが `frameRateNum=0` のとき除算ゼロになる

- Created: 2026-08-28
- Completed: 2026-09-13
- Branch: feature/fix-nvcodec-framerate-div-zero
- Polished: {YYYY-MM-DD}

## 目的

NvCodec ハードウェアエンコーダが VBV バッファサイズを `averageBitRate * frameRateDen / frameRateNum` で計算する。`InitEncode` が `codec_settings->maxFramerate` をそのまま `framerate_` に入れ、0 のまま `CreateEncoder` / `Reconfigure` すると除算ゼロでクラッシュする。これを修正する。

## 現状

- `src/sora-cpp-sdk/src/hwenc_nvcodec/nvcodec_video_encoder.cpp` の `NvCodecVideoEncoderImpl` で `framerate_` の宣言初期値は `0`
- `InitEncode()` が `framerate_ = codec_settings->maxFramerate` とする。0 チェックが無い
- `CreateEncoder()` が `initialize_params.frameRateNum = framerate` とし、`vbvBufferSize = averageBitRate * frameRateDen / frameRateNum` する (`frameRateDen` は 1)
- 再設定経路も `encode_config.rcParams.vbvBufferSize = averageBitRate * 1 / framerate_` する
- `SetRates()` は `parameters.framerate_fps < 1.0` なら return するが、初回 `InitEncode` は通らない

## 設計方針

- `InitEncode` でフレームレートが 1 未満ならエラーを返すか、実装が保証する下限 (1) に丸める。ゼロのまま `CreateEncoder` に渡さない
- `CreateEncoder` / `Reconfigure` の除算前にも `frameRateNum == 0` を拒否する
- 修正は momo と sora-cpp-sdk の両方に入れ、`update-last-updated.sh` で同期する

## 完了条件

- `maxFramerate` が 0 でも除算ゼロでクラッシュしない
- 1 fps 以上の通常設定は従来通りエンコードされる
- 再設定経路でも `framerate_ == 0` の除算が残らない

## 解決方法

現行の momo が利用する libwebrtc (webrtc-build `m150.7871.3.0`、`WEBRTC_COMMIT=1f975dfd761af6e5d76d28333191973b258d82a8`) では、報告された除算ゼロは発生する経路が無いことをソース照合で確認したため、closed にした。

根拠は次の通り。

- momo の Sora シグナリングで `maxFramerate: 0` を受け取ると、`src/sora/sora_client.cpp` の encodings 変換と `src/rtc/rtc_connection.cpp` の `RTCConnection::SetEncodingParameters` を経て `webrtc::RtpEncodingParameters.max_framerate` に 0.0 が入る
- libwebrtc の `media/engine/webrtc_video_engine.cc` `WebRtcVideoSendChannel::WebRtcVideoSendStream::CreateVideoEncoderConfig()` が `encoder_config.simulcast_layers[i].max_framerate` に同値 (0) を設定する
- しかし `video/config/encoder_stream_factory.cc` は、非 simulcast の `CreateDefaultVideoStreams()` で `simulcast_layers[0].max_framerate > 0` の場合のみ採用し、それ以外は `kDefaultVideoMaxFramerate` (60) を使う。simulcast の `OverrideStreamSettings()` も `overrides.max_framerate > 0` の場合のみ反映する。いずれも 0 はエンコーダへ渡らない
- `modules/video_coding/video_codec_initializer.cc` の `SetupCodec()` が `video_codec.maxFramerate` を `streams` の最大値 (>0) に設定し、`video/video_stream_encoder.cc` が `InitEncode(&send_codec_)` を呼ぶため、`NvCodecVideoEncoderImpl::InitEncode()` に 0 が届く経路は無い
- simulcast 経路でも `media/engine/simulcast_encoder_adapter.cc` の `VerifyCodec()` が `maxFramerate < 1` を `WEBRTC_VIDEO_CODEC_ERR_PARAMETER` で拒否する
- 再設定経路は `src/sora-cpp-sdk/src/hwenc_nvcodec/nvcodec_video_encoder.cpp` の `SetRates()` が `parameters.framerate_fps < 1.0` を return しており、`Reconfigure()` の `vbvBufferSize` 除算に 0 は入らない

つまり `nvcodec_video_encoder.cpp` にガードの無い除算式は存在するものの、momo の現行実装では `frameRateNum` (＝ `framerate_`) が 0 になって `CreateEncoder()` / `Reconfigure()` に達しない。完了条件「`maxFramerate` が 0 でも除算ゼロでクラッシュしない」は現行実装では満たされており、修正対象のバグは存在しない。将来 libwebrtc や sora-cpp-sdk の利用方法が変わって 0 が届くようになった時点で改めて対策を検討する。
