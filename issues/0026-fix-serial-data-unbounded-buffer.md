# シリアルデータチャネルの読み書きバッファが無制限に増加する

- Created: 2026-08-19
- Completed: {YYYY-MM-DD}
- Branch: feature/fix-serial-data-unbounded-buffer
- Polished: {YYYY-MM-DD}

## 目的

`SerialDataManager` のシリアル読み込みバッファ (`read_line_buffer_`) が改行 `\n` を含まないデータを受信し続けると無制限に成長し、メモリ枯渇の原因になる。また書き込みバッファ (`write_buffer_`) にもバックプレッシャーがなく、DataChannel からシリアル速度を上回るデータが来ると無制限に増加する。さらに一度読み書きエラーが起きるとポートを閉じるだけで復帰手段が無く、以後の送受信が完全停止する。これを修正する。

## 現状

- `src/serial_data_channel/serial_data_manager.cpp`
  - `OnRead()` (129-144 行): `read_line_buffer_` に無制限に `insert`
  - `SendLineFromSerial()`: `\n` が来ない限り `read_line_buffer_` が成長し続ける
  - `StartWrite()` (160-195 行): `write_buffer_` に無条件で `insert`、`OnBufferedAmountChange` は空実装
  - エラー時に `DoCloseSerial()` のみで `DoRead()` を再開しない

## 設計方針

- `read_line_buffer_` に上限 (例: 数 KB) を設け、超過時はエラーとしてポートを閉じるか、古いデータを破棄する
- `write_buffer_` にバックプレッシャーを実装する (バッファサイズ上限で DataChannel への送信を制御する)
- エラー時はポートを再オープンするか、明示的なエラー通知を行う

## 完了条件

- 改行の無いデータを送り続けてもメモリが無制限に増加しない
- 大量送信時にバッファが肥大化しない
- エラー時に動作が停止したことがユーザーに通知される

## 解決方法

未着手 (PR 作成後に追記する)
