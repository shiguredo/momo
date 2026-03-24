# --log-level の実装とログファイル出力

## 概要

ログレベル制御とログファイル出力が未実装。

## 現状

- `--log-level` は CLI でパースされるが `_log_level` に束縛されるだけ
- `tracing_subscriber::fmt().compact().init()` でフィルターなし、stdout のみ
- momo は `webrtc::FileRotatingLogSink` で `webrtc_logs_*` に 10MB x 10 ファイルのローテーション出力

## 必要な実装

- `--log-level`: verbose / info / warning / error / none を tracing のフィルターに適用
- ログファイル出力: tracing_appender 等でローテーション付きファイル出力 (独自依存の追加が必要、pending)

## 解決方法 (--log-level)

- `init_tracing()` 関数を追加し、サブコマンド処理前に呼び出し
- `--log-level` の値 (verbose/info/warning/error/none) を tracing の EnvFilter に変換
- 各サブコマンドの個別 `tracing_subscriber::fmt().compact().init()` を削除し共通化
- `tracing-subscriber` に `env-filter` feature を追加

## pending (ログファイル出力)

ログファイル出力は `tracing_appender` の依存追加が必要。設計判断のため保留。

Completed: 2026-03-22 (--log-level のみ)

## Reopen: 2026-03-22

`--log-level` は実装済みだが、ログファイル出力 (ローテーション付き) が未実装のまま close されていた。
momo は `FileRotatingLogSink` で 10MB x 10 ファイルのローテーション出力を行っており、この機能が残作業として残っている。
