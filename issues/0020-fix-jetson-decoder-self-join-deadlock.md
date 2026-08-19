# Jetson デコーダが解像度変更時の失敗で自己スレッドを Join してデッドロックする

- Created: 2026-08-19
- Completed: {YYYY-MM-DD}
- Branch: feature/fix-jetson-decoder-self-join-deadlock
- Polished: 2026-08-19

## 目的

Jetson ハードウェアデコーダ (`JetsonVideoDecoder`) の CaptureLoop スレッド内で `SetCapture()` が呼ばれ、その中の `INIT_ERROR` マクロが `Release()` → `JetsonRelease()` → `capture_loop_.Finalize()` を呼ぶ。`Finalize()` は自分自身のスレッドを Join するため、解像度変更イベント後の ioctl 失敗 (ハードウェア異常・デバイス切断) でデッドロックする。これを修正する。

## 現状

- `src/sora-cpp-sdk/src/hwenc_jetson/jetson_video_decoder.cpp` の `JetsonVideoDecoder::CaptureLoop()` が解像度変更イベント検出時に `SetCapture()` を CaptureLoop スレッドから呼ぶ
- `SetCapture()` 内の `INIT_ERROR` マクロが失敗時に `Release()` → `JetsonRelease()` → `capture_loop_.Finalize()` を呼び、CaptureLoop スレッド自身を Join する。行番号は `LAST_UPDATED_SORA_CPP_SDK=bd0af595` 時点のもので、`0013-update-sora-cpp-sdk-sdl3-cli11` で `2026.2.1` に同期した際に要再検証
- `SetCapture()` は CaptureLoop スレッドからのみ呼ばれる経路であり、`JetsonConfigure()` 内の同 `INIT_ERROR` は CaptureLoop 外で正しいクリーンアップを行うため、マクロを分岐する必要がある。sora-cpp-sdk 本体も同構造を持ち、`0013` 同期後に解消済みかをまず確認する
- `SetCapture()` の戻り値 (`WEBRTC_VIDEO_CODEC_ERROR`) は `CaptureLoop` 側で無視され、`SendEOS` の未初期化問題 (`jetson_video_decoder.cpp` の `SendEOS()`) は `0021` に委譲する

## 設計方針

- `0013-update-sora-cpp-sdk-sdl3-cli11` の同期後に sora-cpp-sdk `2026.2.1` の `jetson_video_decoder.cpp` で同問題が解消されているかを確認する。解消されていれば同期で取り込み本 issue は `pending` にする。未解消なら momo 側と sora-cpp-sdk 側の両方で修正し `update-last-updated.sh` で同期する
- `INIT_ERROR` は `JetsonConfigure()` 用に据え置き、`SetCapture()` 用に `Release()` を呼ばない派生マクロを新設するか、`SetCapture()` を `int32_t SetCaptureOnCaptureThread()` に分離して `got_error_ = true; return WEBRTC_VIDEO_CODEC_ERROR;` に統一する。呼び出し側 `CaptureLoop` は戻り値を無視せず `got_error_` を見て即時脱出する
- `SetCapture()` 失敗時は `got_error_` を立てて `CaptureLoop` を即時脱出 (`break` / `return`) し、後段の `dqBuffer` に不正状態で到達しないようにする。`got_error_` は `JetsonConfigure()` 成功時または `JetsonRelease()` 末尾で `false` にリセットする。リソース (`dst_dma_fd_` / `capture_plane`) の後始末は `JetsonRelease()` に委ね、上位へのエラー通知は `CaptureLoop` 脱出で自然に `Release()` が呼ばれる経路を担保する
- `JetsonRelease()` に CaptureLoop スレッド上なら `Finalize()` しないガードを追加する (`capture_loop_` が空でなく current thread が `capture_loop_` なら `Finalize()` をスキップし、リソース解放は外部 `Release()` に委譲する)。自己 Join を防ぐ
- 本 issue では自己 Join の解消のみを扱い、`SendEOS` 未初期化は `0021`、`CHUNK_SIZE` 超過 `memcpy` は `0013` の 13 件確認に委譲する

## 完了条件

- `0013` 同期後に `diff-sora-cpp-sdk.sh` で `SetCapture` / `INIT_ERROR` / `Finalize` 経路が未修正であることを確認した上で本 issue を進める。解消済みなら `pending` にする
- 解像度変更イベント後の ioctl 失敗でデッドロックせず、静的解析で `CaptureLoop` スレッドから `Finalize()` / `Join` に到達する経路が無いことを確認する
- 正常時の解像度変更は従来通り動作する
- エラー時にデコーダが安全に破棄され、クラッシュやハングが無くリソースリークが無い

## 解決方法

未着手 (PR 作成後に追記する)
