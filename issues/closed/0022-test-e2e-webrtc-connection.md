# E2E テスト: WebRTC 接続確立テスト

Created: 2026-03-22
Completed: 2026-05-06
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

## 解決方法

- Ayame モードと Sora モードで `wait_for_connection()` を用いた接続確立テストが既に多数実装済みであることを確認
  - Ayame: `test_ayame_mode_with_codec` / `test_ayame_mode_peer_connection` /
    `test_ayame_mode_direction_sendonly_recvonly` / `test_ayame_mode_direction_sendrecv_default` で
    DTLS/ICE 接続確立 + `outbound-rtp` / `inbound-rtp` の映像/音声トラック存在確認まで実施
  - Sora: `test_sora_mode_*` 各テストで `wait_for_connection()` を呼び出して接続確立を検証
- これにより `WebRtcEngine` のリファクタリング時に接続確立が壊れた場合の回帰検出は十分に機能する状態になった
- P2P モードについては Momo 同士で接続できない構造（HTTP サーバー + ブラウザクライアント）のため、
  本 issue とは別に pytest-playwright を用いた接続確立テストを #0036 で対応する
