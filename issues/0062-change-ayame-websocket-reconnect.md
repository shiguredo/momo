# ayame モードの WebSocket 再接続動作を見直す

- Created: 2026-09-11
- Completed: {YYYY-MM-DD}
- Branch: feature/change-ayame-websocket-reconnect
- Polished: 2026-09-14
- Reporter: @miosakuma

## 目的

ayame モードは WebSocket が切断されると理由を問わず再接続を繰り返し、解消不能なエラーでもプロセスが終了しない。ayame が `reject` (認証エラー等) を返した場合は、サーバからのクローズを契機に 0 秒間隔で再接続するため、ayame に接続と reject を繰り返す負荷をかける。また、0 秒間隔の再接続は ping/pong の維持目的には過剰で、ayame が 60 秒 pong を待つことを踏まえると間隔に余裕がある。接続確立前と確立後で再接続の条件と間隔を整理し、解消不能なエラーでの無限再接続と 0 秒リトライをなくす。

## 現状

- `src/ayame/ayame_client.cpp` の `AyameClient::Connect` は watchdog を 30 秒で有効にして WebSocket に接続する
- `AyameClient::ReconnectAfter` は `retry_count_ * kReconnectIntervalStepSeconds` (10 秒) を `kReconnectIntervalMaxSeconds` (30 秒) でクランプした間隔で watchdog を有効にし、`retry_count_` をインクリメントする。watchdog が発火すると `OnWatchdogExpired` が `Reset()` と `Connect()` を行う
- `AyameClient::OnConnect` は接続エラー時に `ReconnectAfter()` を呼ぶ。`retry_count_` の初期値は 0 のため、初回の再接続間隔は 0 秒になる。以降は 10 秒、20 秒、30 秒 (上限 30 秒) と延びる
- `AyameClient::OnClose` は `retry_count_ = 0` にしてから `ReconnectAfter()` を呼ぶ。WebSocket が閉じられるたびに 0 秒間隔で再接続する
- `AyameClient::OnRead` は `type == "reject"` を処理しない。reject を受信しても読み続け、サーバからのクローズで `OnClose` に至り、0 秒再接続を繰り返す
- `type == "bye"` は `connection_` を破棄して `Close()` を呼び、`OnClose` 経由で 0 秒再接続になる
- `AyameClient::DoIceConnectionStateChange` は ICE 接続が `kIceConnectionFailed` になったとき `Close()` を呼び、`OnClose` 経由で 0 秒再接続になる
- watchdog は `Connect` で 30 秒に設定され、ICE 接続が `kIceConnectionConnected` になると 60 秒へ切り替わる。`type == "ping"` の受信ごとに現在の値でリセットされる
- ayame サーバは WebSocket 接続直後から、register の受理 (accept) と無関係に 5 秒間隔で `type == "ping"` を送り、pong が 60 秒返らないと切断する。デフォルト値は `WebSocketPingIntervalSec` (5 秒) と `WebSocketPongTimeoutSec` (60 秒)。つまり ping の受信は入室の受理を意味しない (OpenAyame/ayame-spec の「ピンポン」、OpenAyame/ayame の `connection.go` の `pingTimer` / `pongTimeoutTimer`)
- `--log-level` のデフォルトは `none` で、接続エラーのログはコンソールに表示されない。`webrtc_logs` ファイルには LS_INFO 以上が出力される
- 過去の変更で「再接続処理の 1 回目を、5 秒後からすぐに実行されるように変更する」が入っており、初回再接続の即時実行は意図的なもの

## 設計方針

- 「接続確立」は `accept` を受信して入室できた (PeerConnection を生成する) ことと定義する。ping の受信は accept の前でも起こるため、接続確立の判定に使わない
- `reject` は再試行で解消しないエラー (認証エラー等) として扱い、`reason` をログに出力してプロセスを終了する。再接続しない
- 接続確立前は WebSocket を再接続しない。接続エラー (WebSocket のハンドシェイク失敗)、切断 (accept 前のクローズ)、ping 途絶 (watchdog 発火) のいずれも、原因をログに出力してプロセスを終了する
- 接続確立後の切断では再接続する。再接続間隔は 0 秒にせず、最小 10 秒から始まる既存の 10 秒ステップ (上限 30 秒) を利用する。0 秒再接続の原因である `OnClose` での `retry_count_ = 0` と、`retry_count_` が 0 のまま開始される間隔計算 (0 * 10 = 0 秒) をなくす
- 接続確立後の ping 途絶 (watchdog 60 秒発火) でも再接続する。既存の `OnWatchdogExpired` による再接続の流れを維持し、間隔 0 秒での `Enable` は行わない
- `bye` は入室済みの接続にのみ送られるため接続確立後の扱いに揃え、0 秒再接続にしない

## 完了条件

- ayame から `reject` を受信した場合、`reason` がログに出力されて momo が終了する (再接続しない)
- accept 受信前の接続エラー・切断・ping 途絶で再接続を繰り返さず、momo が終了する
- WebSocket の切断時に 0 秒間隔の再接続が発生しない
- 接続確立後の切断では、0 秒より長い間隔 (最小 10 秒) で再接続し、接続が復旧する
- 接続確立後の ping 途絶 (watchdog 60 秒発火) でも再接続される
- `bye` を受信したときに 0 秒間隔の再接続が発生しない
- 通常の ayame 接続・切断・再接続のフローが動作する (手動確認)

## 解決方法

{YYYY-MM-DD} に追記する
