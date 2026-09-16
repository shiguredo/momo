# p2p モードの WebSocket シグナリングを廃止する

- Created: 2026-09-11
- Completed: {YYYY-MM-DD}
- Branch: feature/change-p2p-signaling-without-websocket
- Polished: {YYYY-MM-DD}
- Reporter: @voluntas

## 目的

P2P モードは Momo 自身がシグナリングサーバとなり、ブラウザや別の Momo と WebSocket で offer / answer / ICE candidate を交換している。しかしシグナリングの往復は接続確立時の数回だけで、WebSocket の常時接続や ping / 再接続の状態管理に見合う利点がない。HTTP のリクエスト / レスポンスで完結する方式に置き換え、実装と状態管理を単純にする。

置き換え先は WHIP / WHEP とは別の Momo 独自のシグナリング方式を検討する。P2P モードは DataChannel を使いたいため、WHIP / WHEP の仕様に縛られず DataChannel のシグナリングも扱える方式にする。

## 現状

- `src/p2p/p2p_session.cpp` の `P2PSession::OnRead` は `/ws` への upgrade リクエストを `P2PWebsocketSession` に渡し、それ以外は `HandleRequest` で `html/` 配下の静的ファイルを返す
- `src/p2p/p2p_websocket_session.cpp` の `P2PWebsocketSession` は Ayame 互換のシグナリングを WebSocket 上で行う。`register` に `accept` (`isExistUser`) を返し、`offer` に answer を返し、`candidate` / `close` / `bye` / `ping` を処理する。ICE candidate は `OnIceCandidate` から `candidate` として送る。watchdog は 30 秒
- ブラウザ側は `html/p2p.html` が `html/webrtc.js` を読み込む。`html/webrtc.js` は `ws://<host>/ws` に WebSocket で接続し、offer / answer / candidate / close をやり取りし、`serial` という名前の DataChannel を作る
- Momo どうしの接続も `doc/USE_P2P.md` のとおり `ayame --signaling-url ws://<IP>:8080/ws` で同じ WebSocket エンドポイントに接続する
- WebSocket を使うため ping による watchdog と接続状態の管理が必要になっている

## 設計方針

- WebSocket を廃止し、HTTP のリクエスト / レスポンスによるシグナリングに置き換える
- WHIP / WHEP をそのまま採用するのではなく、DataChannel のシグナリングも扱える Momo 独自の方式を検討する。Sora モードの `--data-channel-signaling` のように DataChannel 経由でシグナリングを継続する形も候補とする
- `html/` 配下の静的ファイル配信 (`HandleRequest`) は残す
- ブラウザからの接続と、Momo どうし (`ayame` サブコマンド) の接続の両方を新しい方式に揃えるか、Momo どうしは別方式にするかを決める
- エンドポイントのパス、HTTP メソッド、offer / answer / ICE candidate のやり取り、DataChannel の取り扱いを設計としてまとめる

## 完了条件

- WebSocket を使わず、HTTP のリクエスト / レスポンスだけで接続が確立できる
- `html/p2p.html` からブラウザ経由で接続し、映像と音声が送受信できる
- DataChannel によるメッセージの送受信ができる
- `doc/USE_P2P.md` の記載が新しい方式に更新されている

## 解決方法

未着手 (方式が決まり PR 作成後に追記する)

## pending にした理由

WebSocket を廃止する方針は固まっているが、置き換え先のシグナリング方式が未決定である。WHIP / WHEP とは別の Momo 独自の方式として、DataChannel のシグナリングをどう扱うかを含めて設計を検討する必要があり、方式がまとまるまで実装に着手できないため pending とする。
