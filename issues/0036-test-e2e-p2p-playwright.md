# E2E テスト: P2P モードの WebRTC 接続確立テスト (pytest-playwright)

Created: 2026-05-06
Model: Opus 4.7

## 概要

P2P モードの WebRTC 接続確立を検証する E2E テストが未実装。
Ayame モードと Sora モードでは `wait_for_connection()` を用いた接続テストが
既に整備されているが、P2P モードのみ未対応。

## 根拠

P2P モードは「Momo が HTTP サーバー + WebSocket シグナリング + WebRTC エンドポイント」として
動作し、ブラウザクライアントから接続する構造のため、Momo インスタンス同士では接続できない。
そのため Ayame / Sora モードで採用している「2 ピアを起動して相互接続」方式は P2P モードに適用できず、
ブラウザを介したテストが必要となる。

`html/p2p.html` と `html/webrtc.js` は Momo のリリース成果物に含まれており、
これらが Momo 本体と整合した状態で動くことを保証するテストがないと、
P2P モードのシグナリングプロトコルや HTML/JS 側の変更で接続が壊れても検出できない。

## 現状

- `e2e-tests/test_p2p_mode.py` は起動・メトリクス確認・複数インスタンス起動の 4 テストのみ
- WebRTC 接続確立を検証するテストが存在しない
- `html/p2p.html` / `html/webrtc.js` の動作確認は手動でしか行われていない

## 必要な実装

- pytest-playwright を `e2e-tests/pyproject.toml` の dev 依存に追加
- Chromium をヘッドレスで起動し、`--use-fake-device-for-media-stream` 等のフラグで
  fake メディアを注入する設定を `conftest.py` に用意
- `e2e-tests/test_p2p_mode_browser.py` (仮) に以下のテストを追加
  - P2P モードで Momo を起動し、Playwright で `http://localhost:{port}/p2p.html` を開いて
    `connect` を実行、`wait_for_connection()` で接続確立を確認する
  - 接続後に Momo 側のメトリクスから `outbound-rtp` / `inbound-rtp` の存在と
    `packetsSent` / `packetsReceived` の増加を確認する
  - コーデック (VP8 / VP9 / AV1) ごとの接続確立を確認する
- CI で Chromium のインストール (`playwright install chromium`) を行う手順を整備

## 補足

- `webrtc.js` がブラウザ側でどのような API を公開しているかを確認した上で、
  Playwright の `page.evaluate()` から接続を制御する形にする
- macOS / Linux / Windows ヘッドレス環境での動作確認が必要
- Sora モードの実機接続テスト用 `wait_for_connection()` の枠組みは流用できる
