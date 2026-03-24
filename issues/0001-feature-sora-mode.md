# Sora モードの実装

## 概要

sora_sdk を使用した Sora モードの基本実装は完了済み。残りは未使用パラメータの接続と SoraServer 機能。

## 実装済み

- sora_sdk を使った SoraClient の接続・実行
- WebSocket シグナリング (sora_sdk 内部)
- DataChannel シグナリング (`--data-channel-signaling`)
- マルチストリーム (sora_sdk 内部)
- ロール制御 (sendonly / recvonly / sendrecv)
- メタデータ送信 (`--metadata` JSON)
- サイマルキャスト (`--simulcast`)
- スポットライト (`--spotlight`)
- ビットレート制御 (`--video-bit-rate` / `--audio-bit-rate`)
- WebSocket 切断時の動作制御 (`--ignore-disconnect-websocket`)
- メトリクス stats プロバイダー登録
- 映像キャプチャ (カメラ / libcamera / フェイク)
- 音声キャプチャ (実デバイス / ダミー)
- 複数 signaling URL 対応 (`--signaling-urls url1,url2`)
- disconnect-wait-timeout

## 未実装 (sora_sdk に API なし)

- `--spotlight-number`: sora_sdk の builder に対応メソッドなし
- `--data-channel-signaling-timeout`: sora_sdk の builder に対応メソッドなし

## 未実装 (SoraServer 機能)

- `--port`: SoraServer (HTTP API 経由での接続制御) が未実装
- `--auto`: SoraServer 起動時に即座に接続するフラグ。SoraServer が不要な場合は常に自動接続する (現在の動作)

## 未実装 (その他)

- 自動再接続: sora_sdk に再接続機能があるか要確認
- E2E テスト追加

## 参考

- momo の実装: `src/sora/sora_client.h`, `src/sora/sora_client.cpp`
- MOMO.md の「モード」セクション
