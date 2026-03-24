# Ayame モードの自動再接続

## 概要

momo は watchdog 機構付き自動再接続を実装しているが、momo-rs は未実装。

## 現状

- `src/ayame/mod.rs` でシグナリング接続がクローズされると `break` してそのまま終了する
- 再接続のリトライロジックが存在しない

## 必要な実装

- 接続断検出後の自動再接続
- リトライ間隔の制御
- watchdog 機構

## 参考

- momo の実装: `src/ayame/ayame_client.h`, `src/ayame/ayame_client.cpp`
