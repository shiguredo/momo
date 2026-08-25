# /mute / /mute/status API を呼ぶと Unified Plan の local_streams() 使用でプロセスが落ちる

- Created: 2026-08-19
- Completed: {YYYY-MM-DD}
- Branch: feature/fix-local-stream-empty-crash
- Polished: 2026-08-25
- Milestone: 2026.1.0

## 目的

Sora モードの `/mute` / `/mute/status` API が `RTCConnection::GetLocalStream()` 経由で `connection_->local_streams()` を呼ぶが、momo は Unified Plan (`src/rtc/rtc_manager.cpp` で `SdpSemantics::kUnifiedPlan`) を使用しており、libwebrtc の `PeerConnection::local_streams()` は `RTC_CHECK(!IsUnifiedPlan())` で必ず abort する (RTC_CHECK はビルド種別に関わらず有効)。このため接続確立後の `/mute` / `/mute/status` 呼び出しは全モード (sendrecv / recvonly / デバイス無指定) でプロセスが落ちる。Unified Plan API (`GetSenders()`) でローカルトラックを取得するよう修正する。

## 現状

- `src/rtc/rtc_connection.cpp` の `RTCConnection::GetLocalStream()` が `connection_->local_streams()->at(0)` を呼ぶ。`local_streams()` は Unified Plan では `RTC_CHECK(!IsUnifiedPlan())` により abort する (例外ではないため try/catch では捕捉できない)。`at(0)` の `std::out_of_range` には到達しない
- 呼び出し元は `src/sora/sora_session.cpp` の `SoraSession::OnRead()` 内の `/mute` / `/mute/status` 分岐。`IsAudioEnabled()` / `SetAudioEnabled()` 経由で間接的に `GetLocalStream()` を呼ぶ。`SoraSession::OnRead()` は `boost::beast::http::async_read` のハンドラとして `io_context` 上で実行され try/catch が無い
- `src/sora/sora_session.cpp` の `SoraSession::OnRead()` の `/mute` ハンドラは `recv_json.at("audio").as_bool()` / `at("video")` も try/catch なしで呼び、`0015-fix-signaling-json-exception` では `SoraSession` は `127.0.0.1` bind のためスコープ外としたが、本 issue では `/mute` の JSON 例外も同時に保護する
- `src/sora/sora_client.cpp` の offer 処理は `InitTracks(..., std::nullopt)` のため、`--role recvonly` でも送信トラックが `AddTrack` される。その結果 `GetSenders()` 修正後も送信 track があり `/mute` が `200` になる（本修正で `recvonly` 時のみ role を渡して解消する）
- `src/rtc/rtc_manager.cpp` の `RTCManager::InitTracks()` は recvonly 方向や `audio_track_` と `video_track_` の両方が nullptr のときに AddTrack せず、ローカル送信トラックが存在しない状態になる

## 設計方針

- `RTCConnection` のローカルトラック取得を `local_streams()` (PLAN_B_ONLY、Unified Plan で禁止) から Unified Plan API へ置き換える。`connection_->GetSenders()` を走査し、各 sender の `track()` が nullptr のものはスキップし、`track()` の `kind()` が audio / video のトラックを返す。`GetLocalStream()` は廃止し、`GetLocalAudioTrack()` / `GetLocalVideoTrack()` を `GetSenders()` ベースに書き換える
- `GetLocalAudioTrack()` / `GetLocalVideoTrack()` は該当トラックが無い場合 `nullptr` を返す (既存の nullptr チェーンを維持する)。`SetMediaEnabled()` / `IsMediaEnabled()` は nullptr を許容して false を返す既存実装を維持する
- ハンドラ判定用に `HasLocalSendTrack()` を `public` で追加する（audio または video の送信 track が 1 本以上あるか）
- `src/sora/sora_session.cpp` の `SoraSession::OnRead()` 内の `/mute` / `/mute/status` ハンドラで、`HasLocalSendTrack()` が false の場合は `400 Bad Request` + `application/json` で `{"error": "no local stream"}` を返す (片方のみ存在する場合は従来通り部分的 mute を処理する)。`Util::BadRequest` は text/html のため、`SoraSession::CreateBadRequestWithJSON` を新設して JSON 400 に使う。`sendrecv` 時の成功レスポンス (`200 OK` + `{"audio": bool, "video": bool}`) は維持する
- `/mute` ハンドラの `recv_json.at("audio").as_bool()` / `at("video")` も `try` / `catch (const boost::system::system_error&)` / `catch (const std::exception&)` で包み、キー欠落・型不一致時は `400 Bad Request` を返す。`0015` との責務分担は「`0015` は `SoraClient` / `AyameClient` / `P2PWebsocketSession` の `OnRead` 全般、`0016` は `SoraSession::OnRead` の `/mute` 系を担当」とする
- Sora の offer 処理では `InitTracks` に `std::nullopt` を渡していたため、`--role recvonly` でも `AddTrack` されていた。`recvonly` のときだけ `config_.role` を渡し、送信 track 無しの transceiver だけを作る（`sendonly` / `sendrecv` は従来どおり `nullopt` で影響を限定する）
- sora-cpp-sdk の vendoring 対象には `RTCManager` / `RTCConnection` は含まれず、`src/rtc/` は momo 固有のため、momo 側のコードのみ (`src/rtc/` と `src/sora/`) を修正対象とする

## 完了条件

- recvonly 接続および `audio_track_` と `video_track_` の両方が nullptr の条件 (`--no-video-input-device --no-audio-device` 等) で `/mute` / `/mute/status` を呼んでもプロセスが落ちず、`400 Bad Request` + `{"error": "no local stream"}` が返る。片方のみ無しのケースは部分的 mute が成功することを確認する
- sendrecv 接続ではミュート / アンミュートが `200 OK` で機能する (修正後、Unified Plan で `GetSenders()` 経由のトラック取得が正しく動作すること)
- 手動検証手順を `解決方法` に記載する（E2E は任意。モックやスタブは使わない）

## 解決方法

未着手 (PR 作成後に追記する)
