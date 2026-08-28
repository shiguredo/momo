# Jetson デコーダが `CHUNK_SIZE` を超える入力を検証せず memcpy する

- Created: 2026-08-28
- Completed: {YYYY-MM-DD}
- Branch: feature/fix-jetson-decoder-chunk-size-memcpy
- Polished: {YYYY-MM-DD}

## 目的

Jetson ハードウェアデコーダが出力プレーンへ入力フレームを `memcpy` するとき、プレーン容量 (`CHUNK_SIZE` = 4000000) と `input_image.size()` を比較しない。容量を超えるアクセスユニットでバッファを越境書き込みし、クラッシュ・メモリ破壊する。これを修正する。

## 現状

- `src/sora-cpp-sdk/src/hwenc_jetson/jetson_video_decoder.cpp` で `CHUNK_SIZE` を 4000000 と定義し、`setOutputPlaneFormat(..., CHUNK_SIZE)` に渡す
- 同じファイルのデコード入力経路が `memcpy(buffer->planes[0].data, input_image.data(), input_image.size())` し、続けて `bytesused` に `input_image.size()` を入れる
- `input_image.size()` は外部由来で、プレーン長との比較が無い
- `0022` は V4L2 (`V4L2DecodeConverter::Decode`) の mmap 越境であり、本指摘は Jetson の `NvVideoDecoder` 出力プレーンである
- `hwenc_jetson` は momo 独自コードである

## 設計方針

- コピー前に `input_image.size()` がプレーン容量を超えていないか検証する。容量は `buffer->planes[0]` の確保長、または `CHUNK_SIZE` のうち実装が保証する側を使う
- 超過時は `memcpy` せずエラーを返し、エラーログを出す
- momo のみ修正する

## 完了条件

- `CHUNK_SIZE` (プレーン容量) を超える入力で越境書き込みしない
- 通常サイズの入力は従来通りデコードされる
- 超過時にエラーログが出力される

## 解決方法

未着手 (PR 作成後に追記する)
