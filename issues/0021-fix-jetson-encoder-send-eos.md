# Jetson エンコーダの SendEOS が未初期化の NvBuffer ポインタを参照する

- Created: 2026-08-19
- Completed: 2026-09-02
- Branch: feature/fix-jetson-encoder-send-eos
- Polished: 2026-08-20
- Milestone: 2026.1.0

## 目的

Jetson ハードウェアエンコーダ (`JetsonVideoEncoder`) の `SendEOS()` が `NvBuffer* buffer;` を初期化せずに宣言し、`buffer` は出力キューの `getNumQueuedBuffers() == getNumBuffers()` が真 (キューが満杯) の場合にだけ `dqBuffer()` の out 引数として設定される。キューが満杯でない場合、または `dqBuffer()` が失敗した場合、`buffer` は未初期化のまま `buffer->n_planes` / `buffer->planes[i]` を deref する。`JetsonRelease()` は `encoder_` が非 null のとき常に `SendEOS()` を呼ぶため、エンコード停止時に未定義動作 (クラッシュ・データ破壊) を引き起こしうる経路である。これを修正する。

## 現状

- `src/sora-cpp-sdk/src/hwenc_jetson/jetson_video_encoder.cpp` の `SendEOS()`
  - `NvBuffer* buffer;` を未初期化のまま宣言
  - 出力キューが満杯のときだけ `dqBuffer()` を呼び `buffer` を設定
  - `dqBuffer()` が呼ばれない (キューが満杯でない) 場合、または失敗した場合も `buffer->n_planes` / `buffer->planes[i]` に無条件でアクセス
- `JetsonRelease()` は `encoder_` が非 null のとき常に `SendEOS()` を呼ぶ

## 設計方針

- `SendEOS()` で `dqBuffer()` が成功した場合のみ、その out 引数 (`buffer`) を使用する
- キューが満杯でない場合は、`Encode()` と同様に `getNthBuffer(getNumQueuedBuffers())` でバッファを取得し `v4l2_buf.index` を設定したうえで EOS 処理を完了させる
- `dqBuffer()` が失敗した場合は `buffer` を使用せず、EOS 処理を安全に完了させる

## 完了条件

- エンコード停止時に未定義動作が発生しない
- Jetson でのエンコード停止が正常に完了する

## 解決方法

`SendEOS()` はキューが満杯のときだけ `dqBuffer` し、失敗したら `buffer` を触らず return する。空きがあるときは `getNthBuffer` と index でスロットを決める。`buffer` が null なら return する。`JetsonConfigure()` で `reqbufs(V4L2_MEMORY_DMABUF)` したときだけ `output_use_dmabuf_` を立て、その経路の EOS は `fd` と `V4L2_MEMORY_DMABUF` を付けて `qBuffer` する。MMAP / USERPTR は `NvBuffer` の `bytesused` を 0 にしてから `qBuffer` する。sora-cpp-sdk 本体への反映は、本リポジトリの vendored 修正のみとした。
