# メトリクスレスポンスに libwebrtc フィールドを追加する

## 概要

momo のメトリクスレスポンスには `libwebrtc` フィールドがあるが、momo-rs にはない。

## 現状

- `src/metrics.rs` のレスポンス JSON は `version`, `environment`, `stats` の 3 フィールドのみ
- momo は `"Shiguredo-Build {READABLE_VERSION} ({BUILD_VERSION} {COMMIT_SHORT})"` を返す

## 必要な実装

- shiguredo_webrtc からバージョン情報を取得する方法の調査
- レスポンス JSON に `libwebrtc` フィールドを追加

## 解決方法

- `MetricsState` に `libwebrtc` フィールドを追加
- `shiguredo_webrtc::version()` を使い `"Shiguredo-Build {version}"` 形式で設定
- メトリクス JSON レスポンスに `libwebrtc` フィールドを出力

Completed: 2026-03-22
