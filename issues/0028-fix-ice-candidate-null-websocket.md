# シグナリング切断後に ICE candidate コールバックが ws_ を null deref するレースがある

- Created: 2026-08-19
- Completed: 2026-08-26
- Branch: feature/fix-ice-candidate-null-websocket
- Polished: 2026-08-25
- Milestone: 2026.1.0

## 目的

`SoraClient` / `AyameClient` の `OnIceCandidate` コールバックが `ws_` を null チェックなしで使用する。`ws_` は ioc スレッドの `Close()` や DataChannel シグナリングへの切替 (`switched`) で `nullptr` 化される一方、`OnIceCandidate` は WebRTC のシグナリングスレッドから呼ばれるため、切断中ウィンドウで null deref (または ayame では use-after-free) するレースがある。answer 送信など WebRTC スレッドから `ws_` にアクセスする他のコールバックも同様の問題を持つ。これを修正する。

## 現状

- `src/sora/sora_client.cpp` の `OnIceCandidate()`: `ws_->WriteText()` を null チェックなしで呼ぶ
- 同 `CreateAnswer` の完了コールバック: ioc スレッドへの post 内で `connection_` のみチェックし、`ws_` を null チェックなしで直接 `WriteText` する。`switched` で `ws_` のみ nullptr 化された場合に deref する (`Close()` は `ws_` と `connection_` を両方 nullptr 化するため、`connection_` チェックで弾かれる)。re-offer / update 側は `DoSendUpdate` で DataChannel フォールバック済みだが、初期 answer 側には無い
- `src/ayame/ayame_client.cpp` の `OnIceCandidate`: 同様。ayame は `ws_` が `unique_ptr` で `Reset()` により破棄されるため use-after-free リスクがより高い
- 同 `CreateOffer` の完了コールバック (`on_create_offer`): raw の `this` をキャプチャし、WebRTC シグナリングスレッドから `ws_` へ null チェックなしでアクセスする。破棄との競合で use-after-free になる
- 同 `CreateAnswer` の完了コールバック: sora と違い post せず、WebRTC シグナリングスレッドから `ws_` へ null チェックなしでアクセスする
- `ws_` は `SoraClient::Close()` / `switched` 処理 (sora) で nullptr 化、`AyameClient::Reset()` で破棄される
- 既存の `OnIceConnectionStateChange` (sora / ayame) は `destructed_` チェック + `boost::asio::post` + `shared_from_this()` のパターンを採用しており、本修正の参照点となる

## 設計方針

- WebRTC スレッドから直接呼ばれるコールバック (`OnIceCandidate`、ayame の `CreateOffer` / `CreateAnswer` 完了コールバック) は、`OnIceConnectionStateChange` と同じく `destructed_` チェック後に `boost::asio::post` で ioc スレッドへ移し、`shared_from_this()` でオブジェクトを保持する
- sora の `CreateAnswer` 完了コールバックは既に ioc スレッドへの post 済みのため、post 内の送信を `DoSendUpdate(sdp, "answer")` 相当に置き換える。DataChannel シグナリング利用中は DataChannel 経由、そうでなければ `ws_` 経由で送り、どちらも使えない場合のみ無視する
- ioc スレッド内で送信先を判定する。sora の `OnIceCandidate` は `DoSendUpdate` と同じく、DataChannel シグナリング利用中 (`dc_` と `using_datachannel_` が有効で `dc_->IsOpen("signaling")`) は `SendDataChannel("signaling", ...)` で送信し、`switched` 後に届く ICE candidate も DataChannel 経由で送る。DataChannel が使えない場合は `ws_` が非 nullptr なら `WriteText` し、どちらも利用できない場合のみ無視する
- ayame の `unique_ptr` 破棄 (`Reset`) による use-after-free は、post と破棄が同一 ioc スレッドで直列化されることで防ぐ

## 完了条件

- 切断・切替中に ICE candidate が届いてもクラッシュしない
- 切断・切替後に answer 生成が完了してもクラッシュしない
- `switched` (DataChannel シグナリング) 後に届く ICE candidate が DataChannel 経由で送信され、接続が成立する
- 正常なシグナリングフローは従来通り動作する

## 解決方法

Sora / Ayame とも、WebRTC スレッドから直接呼ばれる送信を `destructed_` 確認のうえ `boost::asio::post` で ioc へ移し、`shared_from_this()` でオブジェクトを保持する。Sora は `DoSendSignaling` を追加し、DataChannel の `signaling` が開いていれば DC、`ws_` があれば WebSocket、どちらも無ければ送らない。`DoSendUpdate` と `OnIceCandidate`、初期 `CreateAnswer` はこの経路に揃えた。Ayame は `CreateOffer` / `CreateAnswer` 完了の `[this]` をやめ、ioc 上で `ws_` が無ければ送らない。CI の momo バイナリで Sora / Ayame の接続と配信を確認した。PR は https://github.com/shiguredo/momo/pull/467 。
