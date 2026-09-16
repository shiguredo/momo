# シリアルデータチャネルのバッファが無制限に増加し、エラー時に通知なく停止する

- Created: 2026-08-19
- Completed: {YYYY-MM-DD}
- Branch: feature/fix-serial-data-unbounded-buffer
- Polished: 2026-08-25

## 目的

`SerialDataManager` のシリアル読み込みバッファ (`read_line_buffer_`) が改行 `\n` を含まないデータを受信し続けると無制限に成長し、メモリ枯渇の原因になる。また DataChannel からシリアル速度を上回るデータが届くと書き込みバッファ (`write_buffer_`) が、シリアルから送信したデータがネットワーク速度を上回ると DataChannel 内部の送信バッファがそれぞれ無制限に増加する。さらに一度読み書きエラーが起きるとポートを閉じるだけで、DataChannel は開いたまま送受信が無音で停止する。これを修正する。

## 現状

- `src/serial_data_channel/serial_data_manager.cpp`
  - `OnRead()`: `read_line_buffer_` に上限なしで `insert` する
  - `SendLineFromSerial()`: `\n` が来ない限り `read_line_buffer_` が成長し続ける
  - `StartWrite()`: `write_buffer_` に上限なしで `insert` する
  - `OnRead()` / `OnWrite()` のエラーパス: `DoCloseSerial()` のみで `DoRead()` を再開せず、以後の送受信が停止する
- `src/serial_data_channel/serial_data_channel.h`
  - `OnBufferedAmountChange()`: 空実装。`Send()` は `data_channel_->buffered_amount()` を確認せず無条件に送信するため、リモートが遅い場合に DataChannel 内部の送信バッファが肥大化する

## 設計方針

- `read_line_buffer_` に上限 (例: 4096 バイト) を設け、超過時は古いデータから破棄して警告をログ出力する。ポートは閉じない
- `write_buffer_` に上限 (例: 4096 バイト) を設け、DataChannel から受信したデータが上限を超える場合は破棄して警告をログ出力する。DataChannel 側の送信を止める仕組みが無いため、破棄によるバックプレッシャーとする
- シリアル → DataChannel 方向は、`SendLineFromSerial()` の送信前に `data_channel_->buffered_amount()` を確認し、上限 (例: 4096 バイト) を超えるチャネルへの送信をスキップして警告をログ出力する。スキップした行は再送せず欠損させる (`read_line_buffer_` からの消去は従来通り行う)。これにより DataChannel 内部バッファの肥大化を防ぐ
- 読み書きエラー時は、エラーと送受信停止を `RTC_LOG(LS_ERROR)` で通知し、`DoCloseSerial()` でポートを閉じ、`serial_data_channels_` の各 DataChannel を `Close()` して送受信停止を対向に通知する。自動再オープンは行わない

## 完了条件

- 改行の無いデータを送り続けても `read_line_buffer_` が上限で抑えられ、メモリが無制限に増加しない
- DataChannel からシリアル速度を上回るデータが届いても `write_buffer_` が上限で抑えられ、肥大化しない
- シリアル → DataChannel 方向でも `buffered_amount()` が上限で抑えられ、DataChannel 内部バッファが肥大化しない
- 読み書きエラー時に、エラーと送受信停止がログと DataChannel の Close で通知される
- 正常なデータの送受信は従来通り動作する

## 解決方法

未着手 (PR 作成後に追記する)
