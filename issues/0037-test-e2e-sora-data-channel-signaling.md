# E2E テスト: Sora モード `--data-channel-signaling` の動作確認

Created: 2026-05-06
Model: Opus 4.7

## 概要

Sora モードの `--data-channel-signaling` フラグに対する E2E テストが未実装。
sora_sdk の `data_channel_signaling()` builder へ値が渡されることまでは
コードリーディングで確認できるが、CLI から指定された値が
実際に sendonly / recvonly ペアの接続確立と RTP 送受信を破壊しないかを
確認する自動テストが存在しない。

## 根拠

- `src/sora/mod.rs:360` で `config.data_channel_signaling` を
  sora_sdk の builder に渡している
- `src/main.rs:1166` 付近で CLI フラグとしてパースし、`parse_bool` を
  通して `Option<bool>` に変換している
- `e2e-tests/momo.py` でも `data_channel_signaling` パラメータを
  サポート済 (Literal["true", "false", "none"])
- 既存の Sora モード E2E テストには本フラグを指定するケースが存在せず、
  今後シグナリング切替ロジックを変更した際に回帰を検出できない

## 現状

- `e2e-tests/test_sora_mode.py` ほか既存の Sora モード E2E テストは
  すべて `--data-channel-signaling` を指定しないデフォルト動作のみを検証
- `Momo` クラスは `data_channel_signaling` 引数を受け取り CLI に伝搬する実装が
  済んでいるため、テスト追加だけで検証可能

## 必要な実装

- `e2e-tests/test_sora_mode_data_channel_signaling.py` を新規追加
- `--data-channel-signaling true` と `--data-channel-signaling false` の
  両ケースについて以下を検証
  - sendonly / recvonly ペアで `wait_for_connection()` が成功する
  - sender 側で `outbound-rtp` の `packetsSent` / `bytesSent` が増加する
  - receiver 側で `inbound-rtp` の `packetsReceived` / `bytesReceived` が増加する
- VP8 ソフトウェアエンコーダ / デコーダを用いる
  (他コーデックは #0024 で別途検証予定のため本テストの範囲外)

## 制約

- DataChannel シグナリングへ実際に切替わったかを Sora 側のメトリクスから
  直接観測する手段がない
- 本テストは「フラグ指定で接続が成立し、RTP 送受信が継続する」レベルの
  リグレッションテストとして位置づける

## 参考

- 実装: `src/sora/mod.rs:360`, `src/main.rs:1166-1213`
- 既存テスト: `e2e-tests/test_sora_mode_sendonly_recvonly.py`
- Sora ドキュメント: <https://sora-doc.shiguredo.jp/SIGNALING>
