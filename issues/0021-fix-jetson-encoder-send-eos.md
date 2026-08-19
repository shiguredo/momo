# Jetson エンコーダの SendEOS が未初期化の NvBuffer ポインタを参照する

- Created: 2026-08-19
- Completed: {YYYY-MM-DD}
- Branch: feature/fix-jetson-encoder-send-eos
- Polished: {YYYY-MM-DD}

## 目的

Jetson ハードウェアエンコーダ (`JetsonVideoEncoder`) の `SendEOS()` が `NvBuffer* buffer;` を初期化せずに宣言し、出力キューの `getNumQueuedBuffers() == getNumBuffers()` が偽 (キューが満杯でない) の場合に `dqBuffer()` が呼ばれず `buffer` が未初期化のまま `buffer->n_planes` を deref する。`JetsonRelease()` は常に `SendEOS()` を呼ぶため、エンコード停止時にほぼ確実に発火する未定義動作 (クラッシュ・データ破壊) の経路である。これを修正する。

## 現状

- `src/sora-cpp-sdk/src/hwenc_jetson/jetson_video_encoder.cpp` の `SendEOS()` (386-410 行)
  - 389 行: `NvBuffer* buffer;` 未初期化
  - 396-401 行: キューが満杯のときだけ `dqBuffer()` を呼び `buffer` を設定
  - 403-405 行: `buffer->n_planes` / `buffer->planes[i]` に無条件でアクセス
- `JetsonRelease()` (364 行) は常に `SendEOS()` を呼ぶ
- 同様の未初期化バッファ問題が `JetsonVideoDecoder::SendEOS()` (jetson_video_decoder.cpp:218-238) にもある

## 設計方針

- `SendEOS()` で `dqBuffer()` が成功した場合のみ `buffer` を使用する
- `dqBuffer()` が呼ばれない / 失敗した場合は EOS 処理を安全に完了させる (qBuffer に空の planes を渡す等)
- デコーダ側の `SendEOS()` も同様に修正する

## 完了条件

- エンコード停止時に未定義動作が発生しない
- Jetson でのエンコード停止が正常に完了する
- デコーダ側の `SendEOS()` も同様に安全になる

## 解決方法

未着手 (PR 作成後に追記する)
