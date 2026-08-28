# エンコーダが未初期化の `configured_bitrate_bps_` と比較しビットレート設定を飛ばす

- Created: 2026-08-28
- Completed: {YYYY-MM-DD}
- Branch: feature/fix-encoder-uninitialized-bitrate
- Polished: {YYYY-MM-DD}

## 目的

Jetson と V4L2 のハードウェアエンコーダが、メンバ `configured_bitrate_bps_` を初期化せずに現在値と比較する。未初期化値と設定ビットレートが偶然一致すると `setBitrate` / `VIDIOC_S_CTRL` をスキップし、エンコーダが意図しないビットレートのまま動く。これを修正する。

## 現状

- `src/sora-cpp-sdk/include/sora/hwenc_jetson/jetson_video_encoder.h` の `configured_bitrate_bps_` は宣言のみで初期化子がない
- `src/sora-cpp-sdk/src/hwenc_jetson/jetson_video_encoder.cpp` のコンストラクタは `configured_framerate_` だけ初期化し、`SetBitrateBps()` で `configured_bitrate_bps_ == bitrate_bps` なら return する
- `src/sora-cpp-sdk/include/sora/hwenc_v4l2/v4l2_h264_encoder.h` も同様に初期化子がない
- `src/sora-cpp-sdk/src/hwenc_v4l2/v4l2_h264_encoder.cpp` のコンストラクタは `configured_framerate_fps_` などを初期化するが `configured_bitrate_bps_` は含めない。`SetBitrateBps()` で同じ比較をする
- `hwenc_jetson` は momo 独自コードである。`hwenc_v4l2` は sora-cpp-sdk 由来である

## 設計方針

- 両ヘッダで `configured_bitrate_bps_` を `0` に初期化する。コンストラクタの初期化リストに書いてもよい
- `SetBitrateBps()` の比較ロジックは維持する。`bitrate_bps < 300000` の早期 return も維持する
- Jetson は momo のみ修正する。V4L2 は momo と sora-cpp-sdk の両方に入れ、`update-last-updated.sh` で同期する

## 完了条件

- `configured_bitrate_bps_` が宣言またはコンストラクタで初期化されている
- 初回の有効ビットレート設定が未初期化比較でスキップされない
- 同じビットレートの再設定は従来通りスキップされる

## 解決方法

未着手 (PR 作成後に追記する)
