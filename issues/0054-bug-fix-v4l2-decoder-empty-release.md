# `V4L2H264Decoder::Release()` が空実装でデコーダ資源を解放しない

- Created: 2026-08-28
- Completed: {YYYY-MM-DD}
- Branch: feature/fix-v4l2-decoder-empty-release
- Polished: {YYYY-MM-DD}

## 目的

WebRTC の `VideoDecoder::Release()` はデコーダを未初期化に戻す契約である。`V4L2H264Decoder::Release()` が何もせず `WEBRTC_VIDEO_CODEC_OK` を返すため、`decoder_` (V4L2 デバイス・バッファ・poll スレッド) が残る。再 `Configure` やプロセス終了まで資源がリークし、二重オープンやハングの原因になる。これを修正する。

## 現状

- `src/sora-cpp-sdk/src/hwenc_v4l2/v4l2_h264_decoder.cpp` の `Release()` が return するだけである
- デストラクタは `Release()` を呼ぶため、オブジェクト破棄でも V4L2 変換器を明示解放しない
- `Configure()` は既存の `decoder_` をリセットせず新しい `V4L2DecodeConverter::Create` で上書きする。古い変換器は shared_ptr の最後の参照次第である
- 同じディレクトリの `V4L2H264Encoder::Release()` や他コーデックの `Release()` は実装側資源を解放している

## 設計方針

- `Release()` で `decoder_.reset()` し、コールバックもクリアする
- `Configure()` の前に `Release()` するか、既存 `decoder_` を先に捨てる
- 修正は momo と sora-cpp-sdk の両方に入れ、`update-last-updated.sh` で同期する

## 完了条件

- `Release()` 後に `decoder_` が空で、V4L2 デバイスが閉じる
- デストラクタ経由でも同じ解放が走る
- `Configure` → `Decode` → `Release` → 再 `Configure` が資源リークなくできる

## 解決方法

未着手 (PR 作成後に追記する)
