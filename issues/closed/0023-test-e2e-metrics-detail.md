# E2E テスト: メトリクス API 統計情報詳細検証

Created: 2026-03-22
Completed: 2026-05-06
Model: Opus 4.6

## 概要

メトリクス API が返す統計情報の詳細検証テストが未実装。

## 根拠

`find_stats()` / `find_all_stats()` / `wait_stats` が `momo.py` に定義済みだが未使用。
メトリクス API のレスポンス構造が momo と互換であることを保証するテストがない。

## 現状

- `e2e-tests/momo.py` に `get_metrics(wait_stats=...)` が定義済み
- `e2e-tests/test_ayame_mode.py` に `find_stats()` / `find_all_stats()` が定義済みだが未使用
- 既存テストはメトリクスの `version` フィールド存在のみ確認

## 必要な実装

- WebRTC 接続後の stats レスポンス構造の検証
- `libwebrtc` フィールドの存在確認 (#0004 実装後)
- `environment` フィールドの形式検証 (#0005 実装後)
- `--metrics-allow-external-ip` の動作テスト (#0003 実装後)

## 解決方法

- `find_stats()` / `find_all_stats()` を `momo.py` に共通ヘルパーとして移動し、`test_ayame_mode.py` から import する形に変更
- `e2e-tests/test_metrics_api.py` に以下のテストを追加
  - `test_metrics_version_format`: `version` が `"WebRTC Native Client Momo {version}"` 形式であること
  - `test_metrics_libwebrtc_format`: `libwebrtc` が `"webrtc-rs {version}"` 形式であること
  - `test_metrics_environment_format`: `environment` が `"[ARCH] OS_DETAIL"` 形式で、ARCH がホストの実行アーキテクチャと一致すること、OS_DETAIL が macOS/Linux ごとに想定された形式であること
  - `test_metrics_allow_external_ip_default_localhost_only`: 未指定時は `127.0.0.1` のみで応答し、非ループバック IP からは接続できないこと
  - `test_metrics_allow_external_ip_enabled`: 指定時は `127.0.0.1` と非ループバック IP の双方から応答すること
  - `test_metrics_stats_structure_after_connection`: ayame モードで 2 ピアを接続し、
    `get_metrics(wait_stats=...)` で `outbound-rtp/video` と `inbound-rtp/video` の到着を待った上で、
    `transport` / `peer-connection` / `candidate-pair` / `local-candidate` / `remote-candidate` /
    `outbound-rtp` / `inbound-rtp` / `codec` の必須フィールドを検証
- 非ループバック IP が取得できない環境ではテストを `pytest.skip` する
