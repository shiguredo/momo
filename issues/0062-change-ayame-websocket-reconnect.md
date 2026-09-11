# ayame モードの WebSocket 再接続動作を見直す

- Created: 2026-09-11
- Completed: {YYYY-MM-DD}
- Branch: feature/change-ayame-websocket-reconnect
- Polished: {YYYY-MM-DD}
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
- 接続確立後は `type == "ping"` の受信ごとに watchdog (60 秒) をリセットする。接続確立前の watchdog は 30 秒
- `--log-level` のデフォルトは `none` で、接続エラーのログはコンソールに表示されない。`webrtc_logs` ファイルには LS_INFO 以上が出力される
- 過去の変更で「再接続処理の 1 回目を、5 秒後からすぐに実行されるように変更する」が入っており、初回再接続の即時実行は意図的なもの

## 設計方針

- `reject` は再試行で解消しないエラー (認証エラー等) として扱い、`reason` をログに出力してプロセスを終了する。再接続しない
- 接続を確立する (ping を受信する) までは WebSocket を再接続しない。接続エラーや切断が起きた場合は原因をログに出力してプロセスを終了する
- 接続確立後の切断では再接続する。間隔は 0 秒ではなく、ayame が 60 秒 pong を待つことを踏まえた値 (既存の 10 秒ステップなど) にする
- `bye` の受信時も 0 秒再接続にしない。再接続するか終了するかは接続確立前後の扱いと揃える

## 完了条件

- ayame から `reject` を受信した場合、`reason` がログに出力されて momo が終了する (再接続しない)
- 接続確立前の接続エラー・切断で再接続を繰り返さない
- WebSocket の切断時に 0 秒間隔の再接続が発生しない
- 接続確立後の切断では、0 秒より長い間隔で再接続し、接続が復旧する
- `bye` を受信したときに 0 秒間隔の再接続が発生しない
- 通常の ayame 接続・切断・再接続のフローが動作する (手動確認)

## 解決方法

{YYYY-MM-DD} に追記する
