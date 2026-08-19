# V4L2 デコーダが入力フレームのサイズ未検証で mmap バッファを越境書き込みする

- Created: 2026-08-19
- Completed: {YYYY-MM-DD}
- Branch: feature/fix-v4l2-decoder-memcpy-overflow
- Polished: {YYYY-MM-DD}

## 目的

Raspberry Pi の V4L2 デコーダ (`V4L2DecodeConverter`) が入力フレームを `memcpy` で src バッファへコピーする際、バッファサイズ (512KB 固定) と入力データサイズの比較が無い。4K 高ビットレート H.264 の I フレームなど 512KB を超える入力で mmap 領域を越境書き込みし、クラッシュ・メモリ破壊する可能性がある。これを修正する。

## 現状

- `src/sora-cpp-sdk/src/hwenc_v4l2/v4l2_converter.cpp` の `V4L2DecodeConverter::Decode()` (699-733 行) が `src_buffers_.at(*index)` の `buffer.planes[0].start` へ `memcpy(buffer.planes[0].start, data, size)` を実行
- src バッファは `InitFormat(..., 512 << 10, &src_fmt)` (534 行) で 512KB に固定
- `size` は `input_image.size()` (`v4l2_h264_decoder.cpp` の `Decode()` から) 由来の未検証の外部値

## 設計方針

- コピー前に `size > バッファの sizeimage` を検証し、超過時はエラーを返す (またはバッファを動的確保する)
- エラー時はデコーダがクラッシュせず、エラーログを出力して失敗を通知する
- バッファサイズの定数を明示的な定数にする

## 完了条件

- 512KB を超える入力フレームでクラッシュ・メモリ破壊しない
- 通常サイズの入力は従来通りデコードされる
- 超過時にエラーが通知される

## 解決方法

未着手 (PR 作成後に追記する)
