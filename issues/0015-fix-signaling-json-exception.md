# シグナリングサーバから受信した JSON のパース例外未処理でプロセスが落ちる

- Created: 2026-08-19
- Completed: {YYYY-MM-DD}
- Branch: feature/fix-signaling-json-exception
- Polished: {YYYY-MM-DD}

## 目的

Sora / Ayame / P2P の各シグナリングクライアントがサーバから受信する JSON メッセージを `boost::json` でパースする際、`parse()` / `at()` / `as_bool()` / `as_string()` が投げる例外を一切 catch していない。サーバが壊れた・悪意のある JSON を送ると `boost::json::system_error` が `io_context::run()` から伝播し `std::terminate` でプロセスが即死する。特に P2P サーバは `0.0.0.0:8080` に認証なしで bind されているため、任意のネットワークユーザーから DoS が可能。これを修正する。

## 現状

- `src/sora/sora_client.cpp` の `OnMessage()` (WebSocket 経由) と DataChannel 経由の `OnMessage()` が `boost::json::parse` / `at()` を try/catch なしで使用
- `src/p2p/p2p_websocket_session.cpp` の `OnMessage()` が `recv_message.at("type")` 等を未処理で使用
- `src/ayame/ayame_client.cpp` の `OnMessage()` が同様

## 設計方針

- シグナリング JSON のパース・アクセスを try/catch で包み、パース失敗・キー欠落・型不一致の場合はエラーをログ出力して、そのメッセージを無視する (プロセスは継続)
- 接続フローが進まない異常メッセージに対しては、既存の watchdog に依存するのではなく、明示的なエラー処理を行う
- 3 クライアント (sora / p2p / ayame) で共通の安全な JSON アクセスヘルパーを導入する

## 完了条件

- 不正な JSON / キー欠落メッセージを送ってもプロセスが落ちない
- 正常なシグナリングフローが従来通り動作する
- 各モードの E2E テストが通る
- 不正 JSON に対するテストが追加されている

## 解決方法

未着手 (PR 作成後に追記する)
