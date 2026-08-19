# recvonly / デバイス無指定時に /mute API を呼ぶとプロセスが落ちる

- Created: 2026-08-19
- Completed: {YYYY-MM-DD}
- Branch: feature/fix-local-stream-empty-crash
- Polished: 2026-08-19
- Milestone: 2026.1.0

## 目的

`RTCConnection::GetLocalStream()` が `connection_->local_streams()->at(0)` を例外チェックなしで呼ぶため、`recvonly` 方向や `--no-video-input-device --no-audio-device` 指定時に `local_streams()` が空だと `std::out_of_range` 例外でプロセスが落ちる。Sora モードの `/mute` API から呼ばれるため、接続確立後に `/mute` を呼ぶと即座にクラッシュする。これを修正する。

## 現状

- `src/rtc/rtc_connection.cpp` の `RTCConnection::GetLocalStream()` が `connection_->local_streams()->at(0)` を未チェックで使用。`GetLocalStream()` は `GetLocalAudioTrack()` / `GetLocalVideoTrack()` からも呼ばれるが、そちらは `nullptr` チェックがあるため `GetLocalStream()` 自体で空チェックが必要。`at(0)` は `std::out_of_range` を投げ、`SoraSession::OnRead()` は `boost::beast::http::async_read` のハンドラとして `io_context` 上で実行され `try/catch` が無いため `std::terminate` に至る
- 呼び出し元は `src/sora/sora_session.cpp` の `SoraSession::OnRead()` 内の `/mute` / `/mute/status` 分岐。`IsAudioEnabled()` / `SetAudioEnabled()` 経由で間接的に `GetLocalStream()` を呼び、null 時の分岐がなく `local_streams` が空の `recvonly` でも `200 OK` で誤った `{"audio": true}` を返す
- `src/sora/sora_session.cpp` の `SoraSession::OnRead()` の `/mute` ハンドラは `recv_json.at("audio").as_bool()` / `at("video")` も `try/catch` なしで呼び、`0015-fix-signaling-json-exception` では `SoraSession` は `127.0.0.1` bind のためスコープ外としたが、本 issue では `/mute` の JSON 例外も同時に保護する
- `src/rtc/rtc_manager.cpp` の `RTCManager::InitTracks()` は `recvonly` 方向や `audio_track_` と `video_track_` の両方が `nullptr` のときに `AddTrack` せず `local_streams()` が空になる。sora-cpp-sdk 側の `RTCManager` も同様の分岐を持つと想定されるが `src/sora-cpp-sdk/` は `0013` で同期されるため本 issue では momo 側のみを対象とする

## 設計方針

- `RTCConnection::GetLocalStream()` で `connection_->local_streams()->count() == 0` を確認し、空の場合 `nullptr` を返す。`at(0)` を呼ばず、例外捕捉ではなく事前 `count()` チェックで安全に取得する。`empty()` の有無は `api/peer_connection_interface.h` の `StreamCollectionInterface` で `count()` が正しいことを確認済みとして `count()` に一本化する。当該 `at(0)` 行は削除して置換する
- `RTCConnection::GetLocalStream()` の可視性を `private` から `public` に変更するか、`HasLocalStream()` を新設してハンドラ側で `nullptr` 判定できるようにする。本 issue では `GetLocalStream()` を `public` にしてハンドラで直接 `if (!rtc_conn->GetLocalStream())` で判定する
- `src/sora/sora_session.cpp` の `SoraSession::OnRead()` 内の `/mute` / `/mute/status` ハンドラで `GetLocalStream()` が `nullptr` の場合は `400 Bad Request` + `application/json` で `{"error": "no local stream"}` を返す。`src/util.h` の `Util::BadRequest` 相当の JSON エラーヘルパーを使い、プロセスは継続する。`sendrecv` 時の成功レスポンス (`200 OK` + `{"audio": bool, "video": bool}`) は維持する。`IsAudioEnabled()` が `nullptr` 時に `false` を返す挙動は利用せず、ハンドラ側で明示的に分岐する
- `/mute` ハンドラの `recv_json.at("audio").as_bool()` / `at("video")` も `try` / `catch (const boost::system::system_error&)` で包み、キー欠落・型不一致時は `400 Bad Request` を返す。`0015` との責務分担は「`0015` は `SoraClient` / `AyameClient` / `P2PWebsocketSession` の `OnRead` 全般、`0016` は `SoraSession::OnRead` の `/mute` 系を担当」とする
- `GetLocalAudioTrack()` / `GetLocalVideoTrack()` は既に `nullptr` 考慮済みのため追加修正不要だが、ハンドラ側の分岐で回帰しないことを確認する
- sora-cpp-sdk 側は参照のみで変更せず、momo 側の `src/rtc/rtc_connection.cpp` のみを修正対象とする

## 完了条件

- `recvonly` 接続および `audio_track_` と `video_track_` の両方が `nullptr` の条件 (`--no-video-input-device --no-audio-device` 等) で `/mute` / `/mute/status` を呼んでもプロセスが落ちず、`400 Bad Request` + `{"error": "no local stream"}` が返る。片方のみ無しのケースは部分的 `mute` が成功することを確認する
- `sendrecv` 接続では従来通りミュート / アンミュートが `200 OK` で機能する
- 手動検証手順を `解決方法` に記載し、E2E は任意とする。`recvonly` と `no-device` の 2 パターンで `400` JSON が返ることを確認するスクリプトで回帰を防止する

## 解決方法

未着手 (PR 作成後に追記する)
