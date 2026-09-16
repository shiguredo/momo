# RTCManager に Factory 単位と Connection 単位の状態が同居している

- Created: 2026-09-11
- Completed: {YYYY-MM-DD}
- Branch: feature/refactor-rtc-manager-state
- Polished: {YYYY-MM-DD}
- Reporter: @tnoho

## 目的

`RTCManager` はプロセスで 1 つ生成され、PeerConnectionFactory やスレッドのように全コネクションで共有するものと、`video_sender_` のようにコネクション単位のものを同じクラスに持っている。共有するものとコネクション単位のものを分離し、`RTCManager` がコネクション単位の状態を持たないようにする。

## 現状

- `src/rtc/rtc_manager.h` の `RTCManager` は `factory_` / `context_` / `network_thread_` / `worker_thread_` / `signaling_thread_` などの Factory 単位のメンバと、`audio_track_` / `video_track_` / `video_sender_` を同じクラスに持つ
- `audio_track_` / `video_track_` は `RTCManager` のコンストラクタで 1 度だけ生成され、`RTCManager::InitTracks` で各コネクションに追加される共有の送信トラック
- `video_sender_` は `RTCManager::InitTracks` でコネクションごとに `AddTrack` の戻り値で上書きされる (`src/rtc/rtc_manager.cpp`)。`RTCManager::SetParameters` は `video_sender_` の `GetParameters` / `SetParameters` を呼ぶ
- Sora / Ayame モードは 1 プロセス 1 コネクションだが、P2P モードは WebSocket 接続ごとに `RTCManager::CreateConnection` と `InitTracks` を呼ぶため、`video_sender_` が最後のコネクションのものに上書きされる。P2P は `SetParameters` を呼ばないため現状は実害が出ていない
- `RTCConnection` には `encodings_` / `mid_` と `SetEncodingParameters` / `ResetEncodingParameters` があり、コネクション単位のパラメータ設定が既に集まっている

## 設計方針

- `video_sender_` を `RTCConnection` に移し、`SetParameters` に相当する処理を `RTCConnection` のメソッドにする。呼び出し元 (`src/sora/sora_client.cpp` / `src/ayame/ayame_client.cpp`) はコネクション経由で呼ぶ
- `RTCManager` には Factory 単位のもの (factory / context / スレッド / config / receiver / data manager) と、全コネクションで共有する送信トラック (`audio_track_` / `video_track_`) だけを残す
- `audio_track_` / `video_track_` を共有のままにするかを確認し、コネクション単位にすべきなら `RTCConnection` 側へ移す。共有で問題ない場合は `RTCManager` に残す
- `InitTracks` がコネクション単位の状態を `RTCManager` に書き戻さないようにする

## 完了条件

- `RTCManager` が `video_sender_` などコネクション単位の状態を持たない
- `RTCManager::SetParameters` に相当する処理が `RTCConnection` に移り、Sora / Ayame モードで従来どおり `DegradationPreference` が設定される
- P2P モードで複数コネクションを張っても、各コネクションの送信トラックの状態が互いに干渉しない
- Sora / Ayame / P2P の接続と配信が従来どおり動作する

## 解決方法

{YYYY-MM-DD} に追記する
