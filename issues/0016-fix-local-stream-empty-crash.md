# recvonly / デバイス無指定時に /mute API を呼ぶとプロセスが落ちる

- Created: 2026-08-19
- Completed: {YYYY-MM-DD}
- Branch: feature/fix-local-stream-empty-crash
- Polished: {YYYY-MM-DD}

## 目的

`RTCConnection::GetLocalStream()` が `connection_->local_streams()->at(0)` を例外チェックなしで呼ぶため、`recvonly` 方向や `--no-video-input-device --no-audio-device` 指定時に `local_streams()` が空だと `std::out_of_range` 例外でプロセスが落ちる。Sora モードの `/mute` API から呼ばれるため、接続確立後に `/mute` を呼ぶと即座にクラッシュする。これを修正する。

## 現状

- `src/rtc/rtc_connection.cpp` の `RTCConnection::GetLocalStream()` が `local_streams()->at(0)` を未チェックで使用
- 呼び出し元は `src/sora/sora_session.cpp` の `/mute/status` / `/mute` ハンドラ
- `src/rtc/rtc_manager.cpp` の `InitTracks()` は `recvonly` 方向やデバイス無指定時にトラックを追加しないため `local_streams()` は空になりうる

## 設計方針

- `GetLocalStream()` で `local_streams()` が空の場合 `nullptr` を返すようにする
- `/mute` / `/mute/status` ハンドラ側で空ストリーム・null を考慮して 404 相当のレスポンスを返す
- `GetLocalAudioTrack()` / `GetLocalVideoTrack()` の null ハンドリングを確認する

## 完了条件

- recvonly 接続で `/mute` / `/mute/status` を呼んでもプロセスが落ちない
- sendrecv 接続では従来通りミュートが機能する
- 関連する E2E テストが追加されている

## 解決方法

未着手 (PR 作成後に追記する)
