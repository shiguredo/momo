# --screen-capture の実装

## 概要

CLI オプションはパースされるが未使用。スクリーンキャプチャ機能。

## 現状

- `src/main.rs` で `_screen_capture` に束縛されるだけ
- momo は WebRTC DesktopCapturer ベース、マルチディスプレイ対応

## 必要な実装

- shiguredo_screen_capture crate の新規作成が必要
  - 各環境でスクリーンキャプチャの API が異なるため、抽象化レイヤーが必要
  - macOS: ScreenCaptureKit
  - Linux (Wayland): PipeWire (xdg-desktop-portal 経由の ScreenCast)
  - Linux (X11): XShm / XComposite (レガシー)
  - Windows: DXGI Desktop Duplication
- マルチディスプレイ対応
- WebRTC VideoTrackSource への統合

## pending 理由

shiguredo_screen_capture crate の新規作成が必要。各環境のスクリーンキャプチャ API が異なるため、設計判断が必要。
