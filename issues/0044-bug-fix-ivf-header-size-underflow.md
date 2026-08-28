# IVF ヘッダ除去がフレームサイズを検証せず符号なし減算がアンダーフローする

- Created: 2026-08-28
- Completed: {YYYY-MM-DD}
- Branch: feature/fix-ivf-header-size-underflow
- Polished: {YYYY-MM-DD}

## 目的

VP8 / VP9 / AV1 のエンコード出力から IVF ファイルヘッダ (32 バイト) とフレームヘッダ (12 バイト) を除く処理が、残サイズを確認せず `size -= 32` / `size -= 12` する。ヘッダより短いパケットで符号なし整数がアンダーフローし、巨大サイズで `EncodedImageBuffer` を作ってクラッシュ・メモリ破壊する。これを修正する。

## 現状

- `src/sora-cpp-sdk/src/hwenc_jetson/jetson_video_encoder.cpp` の `SendFrame()` が `DKIF` なら `buffer += 32; size -= 32;` したあと、無条件に `buffer += 12; size -= 12;` する
- `src/sora-cpp-sdk/src/hwenc_nvcodec/nvcodec_video_encoder.cpp` の AV1 出力処理が同じ減算をする
- `src/sora-cpp-sdk/src/hwenc_vpl/vpl_video_encoder.cpp` の VP9 出力処理が同じ減算をする
- いずれも `DKIF` 判定後も 32 バイト以上あること、フレームヘッダ除去前に 12 バイト以上あることを見ていない
- `hwenc_jetson` は momo 独自、NvCodec / VPL は sora-cpp-sdk 由来である

## 設計方針

- ヘッダ除去の前に残サイズを検証し、不足ならエラーを返してそのフレームを破棄する
- 3 ファイルで同じ検証順序にする (ファイルヘッダ → フレームヘッダ)
- Jetson は momo のみ。NvCodec / VPL は momo と sora-cpp-sdk の両方に入れ、`update-last-updated.sh` で同期する

## 完了条件

- IVF ヘッダより短い出力で `size` がアンダーフローしない
- 通常の IVF 付きフレームは従来通りヘッダを除いてコールバックされる
- 不足時はエラーログを出し、巨大バッファを作らない

## 解決方法

未着手 (PR 作成後に追記する)
