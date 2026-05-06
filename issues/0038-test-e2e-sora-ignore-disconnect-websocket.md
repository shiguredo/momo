# E2E テスト: Sora モード `--ignore-disconnect-websocket` の動作確認

Created: 2026-05-06
Model: Opus 4.7

## 概要

Sora モードの `--ignore-disconnect-websocket` フラグに対する E2E テストが未実装。
sora_sdk の `ignore_disconnect_websocket()` builder へ値が渡されることまでは
コードリーディングで確認できるが、CLI から指定された値が
実際に sendonly / recvonly ペアの接続確立と RTP 送受信を破壊しないかを
確認する自動テストが存在しない。

## 根拠

- `src/sora/mod.rs:363` で `config.ignore_disconnect_websocket` を
  sora_sdk の builder に渡している
- `src/main.rs:1177` 付近で CLI フラグとしてパースし、`parse_bool` を
  通して `Option<bool>` に変換している
- `e2e-tests/momo.py` でも `ignore_disconnect_websocket` パラメータを
  サポート済 (Literal["true", "false", "none"])
- 既存の Sora モード E2E テストには本フラグを指定するケースが存在せず、
  今後接続維持ロジックを変更した際に回帰を検出できない
- 本フラグは `--data-channel-signaling true` 指定時にのみ意味を持つ
  オプションであるため、両者を組み合わせたテストとして整備する

## 現状

- `e2e-tests/test_sora_mode.py` ほか既存の Sora モード E2E テストは
  すべて `--ignore-disconnect-websocket` を指定しないデフォルト動作のみを検証
- `Momo` クラスは `ignore_disconnect_websocket` 引数を受け取り CLI に伝搬する
  実装が済んでいるため、テスト追加だけで検証可能

## 必要な実装

- `e2e-tests/test_sora_mode_ignore_disconnect_websocket.py` を新規追加
- 以下のパラメータ組み合わせで sendonly / recvonly ペアを起動
  - `--data-channel-signaling true --ignore-disconnect-websocket true`
  - `--data-channel-signaling true --ignore-disconnect-websocket false`
- 各ケースで以下を検証
  - sendonly / recvonly ペアで `wait_for_connection()` が成功する
  - sender 側で `outbound-rtp` の `packetsSent` / `bytesSent` が増加する
  - receiver 側で `inbound-rtp` の `packetsReceived` / `bytesReceived` が増加する
- VP8 ソフトウェアエンコーダ / デコーダを用いる
  (他コーデックは #0024 で別途検証予定のため本テストの範囲外)

## 制約

- 本フラグは「WebSocket 切断後も DataChannel 経由でセッションを維持する」
  ことを意図したオプションだが、E2E テストから WebSocket を物理的に切断する
  手段が現状のテスト基盤では用意されていない
- 本テストは「フラグ指定で接続が成立し、RTP 送受信が継続する」レベルの
  リグレッションテストとして位置づける
- 切断挙動を含む本格的なテストは別 issue として今後検討する

## 既知の問題

`ignore_disconnect_websocket=true` ケースは現状 fail するため
`pytest.mark.xfail(strict=True)` を付与する。

- 原因: sora-rust-sdk (`connection.rs:859` 周辺) が WebSocket 受信ループで
  `stream.read()` が返す `UnexpectedEof` (close_notify なしの切断) を
  致命的エラーとして bubble up している
- 期待動作: `--ignore-disconnect-websocket true` 指定時、Sora 側からの
  WebSocket 切断 (close_notify あり / なし問わず) を正常切断として扱い、
  DataChannel 経由でセッションを継続する
- 対応: sora-rust-sdk 側で `UnexpectedEof` を `n == 0` と同等に扱う修正を
  入れ、新バージョンを momo に取り込んだ後に `pytest.mark.xfail` マークを外す
- `strict=True` を付けているため、修正後に xfail を外し忘れると XPASS で
  fail するためバグ修正に気付ける

## 参考

- 実装: `src/sora/mod.rs:363`, `src/main.rs:1177-1216`
- 既存テスト: `e2e-tests/test_sora_mode_sendonly_recvonly.py`
- 関連 issue: #0037 (`--data-channel-signaling` の E2E テスト)
- Sora ドキュメント: <https://sora-doc.shiguredo.jp/SIGNALING>
