# シグナリング切断後に ICE candidate コールバックが ws_ を null deref するレースがある

- Created: 2026-08-19
- Completed: {YYYY-MM-DD}
- Branch: feature/fix-ice-candidate-null-websocket
- Polished: {YYYY-MM-DD}
- Milestone: 2026.1.0

## 目的

`SoraClient` / `AyameClient` の `OnIceCandidate` コールバックが `ws_` を null チェックなしで使用する。`ws_` は ioc スレッドの `Close()` や DataChannel シグナリングへの切替 (`switched`) で `nullptr` 化される一方、`OnIceCandidate` は WebRTC のシグナリングスレッドから呼ばれるため、切断中ウィンドウで null deref (または ayame では use-after-free) するレースがある。answer 送信など WebRTC スレッドから `ws_` にアクセスする他のコールバックも同様の問題を持つ。これを修正する。

## 現状

- `src/sora/sora_client.cpp` の `OnIceCandidate()`: `ws_->WriteText()` を null チェックなしで呼ぶ
- 同 `CreateAnswer` の完了コールバック: `ws_` を null チェックなしで使用 (ioc スレッドへの post 内だが、`Close()` / `switched` で `ws_` のみ null 化される場合に deref する)
- `src/ayame/ayame_client.cpp` の `OnIceCandidate`: 同様。ayame は `ws_` が `unique_ptr` で `Reset()` により破棄されるため use-after-free リスクがより高い
- 同 `CreateOffer` / `CreateAnswer` の完了コールバック: WebRTC シグナリングスレッドから `ws_` へ null チェックなしでアクセスする
- `ws_` は `SoraClient::Close()` / `switched` 処理 (sora) で nullptr 化、`AyameClient::Reset()` で破棄される
- 既存の `OnIceConnectionStateChange` (sora / ayame) は `destructed_` チェック + `boost::asio::post` + `shared_from_this()` のパターンを採用しており、本修正の参照点となる

## 設計方針

- `OnIceConnectionStateChange` と同じく、WebRTC スレッドからの `ws_` アクセスは `destructed_` チェック後に `boost::asio::post` で ioc スレッドへ移し、null チェックを追加する
- `ws_` が nullptr の場合、そのコールバックを無視する
- ayame の `unique_ptr` 破棄 (`Reset`) による use-after-free は、post と破棄が同一 ioc スレッドで直列化されることで防ぐ

## 完了条件

- 切断・切替中に ICE candidate が届いてもクラッシュしない
- 切断・切替後に answer 生成が完了してもクラッシュしない
- 正常なシグナリングフローは従来通り動作する

## 解決方法

未着手 (PR 作成後に追記する)
