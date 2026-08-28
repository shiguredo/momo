# V4L2 デコーダが入力フレームのサイズ未検証で mmap バッファを越境書き込みする

- Created: 2026-08-19
- Completed: 2026-08-28
- Branch: feature/fix-v4l2-decoder-memcpy-overflow
- Polished: 2026-08-20
- Milestone: 2026.1.0

## 目的

Raspberry Pi の V4L2 デコーダ (`V4L2DecodeConverter`) が入力フレームを `memcpy` で src バッファへコピーする際、バッファ容量 (mmap 実長) と入力データサイズの比較が無い。高ビットレート 4K H.264 の I フレームなど、バッファ容量を超える入力で mmap 領域を越境書き込みし、クラッシュ・メモリ破壊する可能性がある。これを修正する。

## 現状

- `src/sora-cpp-sdk/src/hwenc_v4l2/v4l2_converter.cpp` の `V4L2DecodeConverter::Decode()` が `src_buffers_.at(*index)` の `buffer.planes[0].start` へ `memcpy(buffer.planes[0].start, data, size)` を実行
- src バッファは `V4L2DecodeConverter::Init()` の `V4L2Helper::InitFormat(..., 512 << 10, &src_fmt)` で sizeimage に 512KB を要求し、`V4L2Buffers::Allocate()` で mmap 確保される。実容量は `VIDIOC_QUERYBUF` 由来の `buffer.planes[0].length` で決まる
- `buffer.planes[0].sizeimage` は `Decode()` 内で入力サイズに上書きされるため容量を表さない。容量の比較対象は `buffer.planes[0].length`
- `size` は未検証の外部値で、次の 2 経路から渡される
  - `v4l2_h264_decoder.cpp` の `V4L2H264Decoder::Decode()` が `input_image.size()` から渡す
  - `v4l2_h264_encoder.cpp` の MJPEG 入力パスが `native_buffer->size()` から渡す
- 変更対象は sora-cpp-sdk から vendoring されたファイルであり、0013 (sora-cpp-sdk 2026.2.1 同期) と並走する。0013 は 0022 を同期確認対象から除外しているため、本修正を先に適用すると 0013 の同期で上書きされても検出されない。0013 の同期を先に完了させてから本修正を適用する

## 設計方針

- コピー前に `size > buffer.planes[0].length` (mmap 実容量) を検証し、超過時はエラーを返す
- エラー時は `PopAvailableBufferIndex()` で取得済みのインデックスをキューへ戻し、バッファが枯渇しないようにする。`V4L2Runner` にはインデックスを返却する公開 API が無いため、`src/sora-cpp-sdk/src/hwenc_v4l2/v4l2_runner.cpp` / `src/sora-cpp-sdk/include/sora/hwenc_v4l2/v4l2_runner.h` への追加も変更対象に含める
- エラー時はデコーダがクラッシュせず、エラーログを出力して失敗を通知する。呼び出し側 (`V4L2H264Decoder::Decode()` / MJPEG 入力パス) は戻り値を握りつぶすため、通知手段はエラーログ出力のみである
- デコーダの src バッファサイズ要求値 (`512 << 10`) を明示的な定数にする
- 0013 の同期完了後に `V4L2DecodeConverter::Decode()` の `memcpy` が未修正のままであることを確認する。上流で解消済みの場合は修正せず pending にする。修正する場合は momo 側に加えて sora-cpp-sdk 側にも反映し、`update-last-updated.sh` で同期する

## 完了条件

- バッファ容量 (mmap 実長) を超える入力フレームでクラッシュ・メモリ破壊しない
- 通常サイズの入力は従来通りデコードされる
- 超過エラーが発生した後も、以降の通常サイズ入力がデコードされ続ける
- 超過時にエラーログが出力される

## 解決方法

`V4L2DecodeConverter::Decode()` で `memcpy` の前に入力 `size` と mmap 実長 `planes[0].length` を比較する。超過時は `V4L2Runner::PushAvailableBufferIndex` でインデックスを戻し、英語のエラーログを出して `WEBRTC_VIDEO_CODEC_ERROR` を返す。入力の sizeimage 要求値は `SRC_BUFFER_SIZEIMAGE` にした。`V4L2H264Decoder::Decode()` は変換器の失敗を返す。MJPEG 経路は従来どおり戻り値を使わない。sora-cpp-sdk 本体への反映と `update-last-updated.sh` は、本リポジトリの vendored 修正のみとした。
