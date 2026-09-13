# IVF ヘッダ除去がフレームサイズを検証せず符号なし減算がアンダーフローする

- Created: 2026-08-28
- Completed: {YYYY-MM-DD}
- Branch: feature/fix-ivf-header-size-underflow
- Polished: 2026-09-13

## 目的

VP8 / VP9 / AV1 のエンコード出力から IVF ファイルヘッダ (32 バイト) とフレームヘッダ (12 バイト) を除く処理が、残サイズを確認せず `size -= 32` / `size -= 12` する。ヘッダより短いパケットで、Jetson / NvCodec は `size_t` の符号なしアンダーフロー、VPL は `int` の負値から `size_t` 変換で巨大な値になり、`EncodedImageBuffer` を巨大サイズで作ってクラッシュ・メモリ破壊する。これを修正する。

## 現状

- `src/sora-cpp-sdk/src/hwenc_jetson/jetson_video_encoder.cpp` の `SendFrame()` が `DKIF` なら `buffer += 32; size -= 32;` したあと、無条件に `buffer += 12; size -= 12;` する
- `src/sora-cpp-sdk/src/hwenc_nvcodec/nvcodec_video_encoder.cpp` の AV1 出力処理が同じ減算をする
- `src/sora-cpp-sdk/src/hwenc_vpl/vpl_video_encoder.cpp` の VP9 出力処理が同じ減算をする
- Jetson / NvCodec の `size` は `size_t`、VPL の `size` は `int` である
- いずれも `DKIF` 判定後も 32 バイト以上あること、フレームヘッダ除去前に 12 バイト以上あることを見ていない
- `hwenc_jetson` は momo 独自、NvCodec / VPL は sora-cpp-sdk 由来である

## 設計方針

- 検証は 3 ファイルで共通の次の順序で行う:
  1. ファイルヘッダの判定前: `size < 4` は DKIF 判定の 4 バイトを読めないためエラー
  2. ファイルヘッダの除去前: `DKIF` に一致した場合、`size < 32` はエラー
  3. フレームヘッダの除去前: `size < 12` はエラー
- 不足・不正時はエラーログを英語で出力し、`WEBRTC_VIDEO_CODEC_ERROR` を返してそのフレームを破棄する
- 検証はファイルヘッダ → フレームヘッダの順に適用する (DKIF でなければ 32 バイトの除去はしない)
- Jetson の AV1 経路ではヘッダ除去後に `buffer[2]` / `buffer[3]` を読む。この読み取りと残サイズの検証は 0046 (Jetson エンコーダが AV1 の OBU サイズを 1 バイト固定で読みバッファを越境する) の対象とし、本 issue はヘッダ除去とその前の検証までを扱う
- Jetson は momo のみ。NvCodec / VPL は momo と sora-cpp-sdk の両方に入れ、`update-last-updated.sh` で同期する

## 完了条件

- IVF ヘッダより短い出力で `size` がアンダーフローしない
- 通常の IVF 付きフレームは従来通りヘッダを除いてコールバックされる
- 不足時はエラーログを出し、巨大バッファを作らない

## 解決方法

未着手 (PR 作成後に追記する)
