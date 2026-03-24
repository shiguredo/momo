# --metrics-allow-external-ip の実装

## 概要

CLI オプションはパースされるが、メトリクスサーバーは常に `0.0.0.0` でバインドしており制御できない。

## 現状

- `src/main.rs` で `_metrics_allow_external_ip` に束縛されるだけで未使用
- `src/metrics.rs` の `TcpListener::bind(("0.0.0.0", port))` が固定

## 必要な実装

- 未指定時は `127.0.0.1` でバインド
- 指定時は `0.0.0.0` でバインド
- `_metrics_allow_external_ip` の値を `MetricsState` または `metrics::run()` に渡す

## 解決方法

- `metrics::run()` に `allow_external_ip: bool` 引数を追加
- `allow_external_ip` が `true` なら `0.0.0.0`、`false` なら `127.0.0.1` でバインド
- `main.rs` の `_metrics_allow_external_ip` からアンダースコアを外し、`metrics::run()` に渡すように変更

Completed: 2026-03-22
