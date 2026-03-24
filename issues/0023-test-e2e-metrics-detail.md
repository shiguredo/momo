# E2E テスト: メトリクス API 統計情報詳細検証

Created: 2026-03-22
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
