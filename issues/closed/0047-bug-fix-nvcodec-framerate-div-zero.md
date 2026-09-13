# NvCodec エンコーダが `frameRateNum=0` のとき除算ゼロになる

- Created: 2026-08-28
- Completed: {YYYY-MM-DD}
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

未着手 (PR 作成後に追記する)
