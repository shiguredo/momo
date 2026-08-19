# シグナリング切断後に ICE candidate コールバックが ws_ を null deref するレースがある

- Created: 2026-08-19
- Completed: {YYYY-MM-DD}
- Branch: feature/fix-ice-candidate-null-websocket
- Polished: {YYYY-MM-DD}
- Milestone: 2026.1.0

## 目的

`SoraClient` / `AyameClient` の `OnIceCandidate` コールバックが `ws_` を null チェックなしで使用する。`ws_` は ioc スレッドの `Close()` や DataChannel シグナリングへの切替 (`switched`) で `nullptr` 化される一方、`OnIceCandidate` は WebRTC のシグナリングスレッドから呼ばれるため、切断中ウィンドウで null deref (または ayame では use-after-free) するレースがある。これを修正する。

## 現状

- `src/sora/sora_client.cpp` の `OnIceCandidate()` (732-737 行): `ws_->WriteText()` を null チェックなしで呼ぶ
- 同 `OnCreateAnswer` コールバック (531-539 行): `ws_` を null チェックなしで使用
- `src/ayame/ayame_client.cpp` の `OnIceCandidate` (529-541 行) / `OnCreateAnswer` 相当 (474-496 行): 同様。ayame は `ws_` が `unique_ptr` で `Reset()` により破棄されるため use-after-free リスクがより高い
- `ws_` は `Close()` (sora_client.cpp:88) / `switched` 処理 (619 行) で nullptr 化される

## 設計方針

- `ws_` のアクセスを ioc スレッドで post して行い、null チェックを追加する
- `ws_` が nullptr の場合、そのコールバックを無視する
- ayame の `unique_ptr` 破棄 (Reset) による use-after-free を防ぐため、破棄とコールバックの同期を取る

## 完了条件

- 切断・切替中に ICE candidate が届いてもクラッシュしない
- 正常なシグナリングフローは従来通り動作する

## 解決方法

未着手 (PR 作成後に追記する)
