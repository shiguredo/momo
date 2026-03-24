# E2E テスト: WebRTC 接続確立テスト

Created: 2026-03-22
Model: Opus 4.6

## 概要

WebRTC 接続が実際に確立されることを検証する E2E テストが未実装。

## 根拠

`wait_for_connection()` が `momo.py` に定義済みだが未使用。
P2P / Ayame モードで接続確立の回帰テストがないため、リファクタリング時に接続が壊れても検出できない。

## 現状

- `e2e-tests/momo.py` に `wait_for_connection()` が定義済みだが呼ばれていない
- 既存の E2E テスト:
  - test_p2p_mode.py: 起動・メトリクス確認・複数インスタンス (4 テスト)
  - test_ayame_mode.py: 起動・設定バリエーション・不正コーデック検証 (5 テスト)
  - test_momo_validation.py: モード固有オプション検証 (4 テスト)

## 必要な実装

- P2P モードでの WebRTC 接続確立テスト
- Ayame モードでの WebRTC 接続確立テスト (Ayame サーバーが必要)
- 接続後の映像/音声トラック存在確認
