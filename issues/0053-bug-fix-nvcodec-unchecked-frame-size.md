# NvCodec エンコーダがフレームサイズを検証せず GPU バッファへコピーする

- Created: 2026-08-28
- Completed: {YYYY-MM-DD}
- Branch: feature/fix-nvcodec-unchecked-frame-size
- Polished: 2026-09-13

## 目的

NvCodec エンコーダが入力フレームを NVENC バッファへコピーするとき、フレームの幅・高さがエンコーダ初期化時の `width_` / `height_` と一致するか見ていない。フレームが初期化サイズより大きいと GPU / ステージングバッファへ越境書き込みし、Linux の NV12 経路では小さいフレームでコピー元を越境読み出しする。クラッシュ・映像破壊の原因になる。これを修正する。

## 現状

- `src/sora-cpp-sdk/src/hwenc_nvcodec/nvcodec_video_encoder.cpp` の `Encode()` が Windows で `libyuv::NV12Copy` / `I420ToNV12` にコピーサイズとして `frame_buffer->width()` / `height()` を渡す。コピー先は `width_` / `height_` で確保したステージングテクスチャで、UV プレーン先頭は `map.pData + height_ * map.RowPitch` と `height_` を使うため、フレームが大きいと越境する
- Linux では `cuda_->Copy()` に、NV12 フレームは `width_` / `height_` を、I420 フレームは `frame_buffer->width()` / `height()` を渡す
- `pic_params.inputWidth` / `inputHeight` は `width_` / `height_` である。フレーム実寸との照合が無い
- エンコーダ作成時のテクスチャ / CUDA バッファは `width_` / `height_` で確保される

## 設計方針

- `Encode()` でコピー処理に入る前に、フレームの幅・高さが `width_` / `height_` と一致するか検証する。不一致ならコピーせず `WEBRTC_VIDEO_CODEC_ERROR` を返し、英語のエラーログを出力する
- 解像度変更への対応（再初期化）は本 issue では行わない。一致しないフレームを拒否して越境コピーを止めることだけを扱う
- 修正は momo と sora-cpp-sdk の両方に入れ、`update-last-updated.sh` で同期する

## 完了条件

- 初期化サイズと異なるフレームで GPU / ステージングへ越境コピーしない
- 一致するフレームは従来通りエンコードされる
- 不一致時にエラーログが出力される

## 解決方法

未着手 (PR 作成後に追記する)
