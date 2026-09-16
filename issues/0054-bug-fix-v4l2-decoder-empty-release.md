# `V4L2H264Decoder::Release()` が空実装でデコーダ資源を解放しない

- Created: 2026-08-28
- Completed: {YYYY-MM-DD}
- Branch: feature/fix-v4l2-decoder-empty-release
- Polished: 2026-09-13

## 目的

WebRTC の `VideoDecoder::Release()` はデコーダを未初期化に戻す契約である。`V4L2H264Decoder::Release()` が何もせず `WEBRTC_VIDEO_CODEC_OK` を返すため、Release() 後も `decoder_` (V4L2 デバイス・バッファ・poll スレッド) が残り、デコーダはデコード可能な状態のままになる。WebRTC がデコーダインスタンスを保持したまま `Configure()` をやり直す場合、Release() で資源が解放されず、解放時期がインスタンス破棄までずれ込む。これを修正する。

## 現状

- `src/sora-cpp-sdk/src/hwenc_v4l2/v4l2_h264_decoder.cpp` の `Release()` が `return WEBRTC_VIDEO_CODEC_OK;` するだけである
- デストラクタ (`~V4L2H264Decoder`) は `Release()` を呼ぶが、空実装なので未初期化には戻らない。変換器の破棄は `decoder_` のメンバ破棄で行われるため、`Release()` 単独では解放されない
- `Configure()` は既存の `decoder_` をリセットせず新しい `V4L2DecodeConverter::Create` の結果で上書きする。変換器を参照する shared_ptr は `decoder_` のみ (`V4L2Runner::Create()` の poll スレッドは raw ポインタをキャプチャするだけ) なので、上書き時には旧変換器が破棄される。問題は `Release()` 時点で明示的に破棄する操作が無いことである
- 同じディレクトリの `V4L2H264Encoder::Release()` や他コーデックの `Release()` (`VplVideoDecoderImpl::Release()` / `JetsonVideoDecoder::Release()` / `NvCodecVideoDecoder::Release()`) は実装側資源を解放している

## 設計方針

- `Release()` で `decoder_.reset()` し、未初期化に戻す。`Decode()` は先頭の `decoder_ == nullptr` チェックで `WEBRTC_VIDEO_CODEC_UNINITIALIZED` を返す
- `decode_complete_callback_` はクリアしない。クリアすると Release() 後の再 `Configure()` → `Decode()` が `decode_complete_callback_ == NULL` チェックで `WEBRTC_VIDEO_CODEC_UNINITIALIZED` を返し、再構成後のデコードが失敗する。`RegisterDecodeCompleteCallback()` の再登録が呼び出し側から保証されないためである。上記の他コーデックの `Release()` もコールバックをクリアしていない
- `Configure()` は既存の `decoder_` を先に `Release()` で破棄してから `V4L2DecodeConverter::Create` する。生成失敗時に旧変換器を残さないためである
- 修正は momo と sora-cpp-sdk の両方に入れ、`update-last-updated.sh` で同期する

## 完了条件

- `Release()` 後に `decoder_` が空になり、V4L2 デバイスが閉じる
- `Release()` 後の `Decode()` は `WEBRTC_VIDEO_CODEC_UNINITIALIZED` を返す
- デストラクタ経由でも同じ解放が走る
- `Configure` → `Decode` → `Release` → 再 `Configure` → `Decode` が資源リークなくできる

## 解決方法

未着手 (PR 作成後に追記する)
