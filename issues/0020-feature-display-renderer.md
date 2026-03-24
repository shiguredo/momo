# 表示機能 (Raw プレイヤー) の実装

## 概要

受信映像の表示機能が未実装。

## 現状

- `--use-raw-player`, `--fullscreen`, `--window-width`, `--window-height` は CLI のみ
- momo は SDL2 で受信映像を表示

## 必要な実装

- raw-player-rs (raw_player crate) を使用した映像表示 (SDL3 ベース)
- `--use-raw-player`: Raw プレイヤーでの映像表示
- `--fullscreen`: フルスクリーン表示
- `--window-width` / `--window-height`: ウィンドウサイズ指定
