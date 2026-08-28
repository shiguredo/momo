# NvCodec エンコーダがフレームサイズを検証せず GPU バッファへコピーする

- Created: 2026-08-28
- Completed: {YYYY-MM-DD}
- Branch: feature/fix-nvcodec-unchecked-frame-size
- Polished: {YYYY-MM-DD}

## 目的

NvCodec エンコーダが入力フレームを NVENC バッファへコピーするとき、フレームの幅・高さがエンコーダ初期化時の `width_` / `height_` と一致するか見ていない。より大きいフレームで GPU / ステージングバッファを越境し、クラッシュ・映像破壊する。これを修正する。

## 現状

- `src/sora-cpp-sdk/src/hwenc_nvcodec/nvcodec_video_encoder.cpp` の `Encode()` が Windows で `libyuv::NV12Copy` / `I420ToNV12` に `frame_buffer->width()` / `height()` を渡し、行ピッチ計算は `height_` を使う
- Linux では `cuda_->Copy(..., frame_buffer->width(), frame_buffer->height())` する経路と、`width_` / `height_` を渡す経路がある
- `pic_params.inputWidth` / `inputHeight` は `width_` / `height_` である。フレーム実寸との照合が無い
- エンコーダ作成時のテクスチャ / CUDA バッファは `width_` / `height_` で確保される

## 設計方針

- コピー前にフレームの幅・高さが `width_` / `height_` と一致するか検証する。不一致ならエラーを返しコピーしない
- 解像度変更が必要なら既存の再初期化経路に乗せる。本 issue では越境コピーを止めることだけを扱う
- 修正は momo と sora-cpp-sdk の両方に入れ、`update-last-updated.sh` で同期する

## 完了条件

- 初期化サイズと異なるフレームで GPU / ステージングへ越境コピーしない
- 一致するフレームは従来通りエンコードされる
- 不一致時にエラーログが出力される

## 解決方法

未着手 (PR 作成後に追記する)
