# シグナリングサーバから受信した JSON のパース例外未処理でプロセスが落ちる

- Created: 2026-08-19
- Completed: 2026-08-24
- Branch: feature/fix-signaling-json-exception
- Polished: 2026-08-20
- Milestone: 2026.1.0

## 目的

Sora / Ayame / P2P の各シグナリングでサーバから受信する JSON メッセージを `boost::json` でパースする際、`parse()` / `at()` / `as_bool()` / `as_string()` が投げる `boost::system::system_error` を catch していない。サーバが壊れた・悪意のある JSON を送ると例外が `io_context::run()` から伝播し `std::terminate` でプロセスが即死する。Sora / Ayame はシグナリングサーバからの攻撃、P2P は `src/main.cpp` が `0.0.0.0:8080` (`p2p` サブコマンドの `--port` デフォルト `8080`) に認証なしで bind するため任意のネットワークユーザーからの DoS が可能。これを修正する。

## 現状

- `src/sora/sora_client.cpp` の `SoraClient::OnRead()` が `boost::json::parse()` と `json_message.at()` / `as_string()` / `as_bool()` / `as_array()` を try/catch なしで呼び出している (`CreateRTCConnection` の `jconfig.at("iceServers")` 等を含む)。`SoraClient::OnMessage()` の DataChannel 経由は `parse(data, ec)` でパースエラーはハンドリングしているが、`json.at("type")` 以降のキーアクセスは例外未処理のまま
- `src/p2p/p2p_websocket_session.cpp` の `P2PWebsocketSession::OnRead()` が `recv_message.at("type")` / `at("sdp")` / `ice.at()` 等を未処理で使用。`parse(recv_string, jec)` の `error_code` オーバーロードでパース失敗は握り潰すが `at()` は裸のまま
- `src/ayame/ayame_client.cpp` の `AyameClient::OnRead()` が同様に `parse()` / `at()` を try/catch なしで使用
- sora-cpp-sdk 側の `SoraSignaling` も同様の構造を持つが、momo の `src/sora/sora_client.cpp` は sora-cpp-sdk とは独立した実装であり (DataChannel 圧縮・watchdog 処理を持つ)、momo 側で個別に修正が必要
- `src/sora/sora_session.cpp` の `SoraSession::OnRead()` の `/mute` ハンドラも `recv_json.at("audio").as_bool()` / `at("video")` を try/catch なしで呼ぶが、`SoraSession` は `127.0.0.1` bind のため本 issue ではスコープ外とし、`0016-fix-local-stream-empty-crash` で対応する

## 設計方針

- シグナリング JSON のパース・アクセスを `try` / `catch (const boost::system::system_error&)` / `catch (const std::exception&)` で包み、パース失敗・キー欠落・型不一致の場合はエラーを英語でログ出力してそのメッセージを無視する。catch 後の受信継続はクライアントごとに扱いが異なる: Sora / Ayame の `OnRead` (WS 受信) は catch 内で `DoRead()` → `return` する (P2P の `Guard` パターンは Sora の redirect 経路で二重 Read になるため適用しない)、P2P の `OnRead` は既存 `Guard` が `DoRead()` を保証するため catch 内ではログ出力のみ (catch 内で `DoRead()` を呼ぶと二重 Read になる)、Sora の `OnMessage` (DataChannel 受信) は WS の `DoRead()` の文脈ではないため catch 内ではログ出力のみ。`boost::json::parse()` の `error_code` オーバーロードを使う箇所 (P2P の `parse(recv_string, jec)` 等) も、パース失敗時にログを出力し、`at()` / `as_*()` の例外は同様に catch する
- 異常メッセージで接続フローが進まない場合は、Sora / Ayame は既存の `Watchdog` のタイムアウト (Sora 30/60 秒、Ayame 30/60 秒) で再接続に至る。P2P の watchdog は ping 送信によるセッション維持のみで再接続しないため (register 受信後に有効化)、異常メッセージの回復は catch 後の `DoRead()` 継続が主体となる。watchdog 自体の挙動は変えず、ログ出力で原因追跡可能にする
- 3 クライアント (Sora / P2P / Ayame) の `OnRead` / `OnMessage` で JSON のパース・アクセス箇所を try/catch で包んでクラッシュを止めることを最優先とする。Sora の offer 分岐は `SetOffer()` の非同期コールバック (`boost::asio::post` された lambda) 内でも `json_message` の `at()` / `as_*()` にアクセスする (encodings / mid の取り出し) ため、そのコールバック内にも try/catch を追加する (同期部の try/catch では捕捉できない。コールバック内の catch は WS の `DoRead()` の文脈にないためログのみとする)。共通の安全な JSON アクセスヘルパー (`find()` と型チェック) への置換は本 issue では行わない
- sora-cpp-sdk 側の修正を参考にするが、momo 側の `src/sora-cpp-sdk/` は `0013-update-sora-cpp-sdk-sdl3-cli11` で同期されるため、本 issue では `src/sora/` / `src/ayame/` / `src/p2p/` の momo 固有コードを対象とする。`0029-fix-zlib-uncompress-memory` の `ZlibHelper::Uncompress` 例外とは catch のスコープを分け、JSON 用 try/catch の開始位置は `ZlibHelper::Uncompress()` 呼び出しより後とする (OnMessage では Uncompress が JSON パースより先に呼ばれる)

## 完了条件

- 不正な JSON / キー欠落 / 型不一致メッセージを送ってもプロセスが落ちず、英語でエラーログが出力され受信が継続する
- 正常なシグナリングフロー (Sora / Ayame / P2P の offer / answer / candidate / ping / switched) が従来通り動作する
- 各モードの E2E テスト (`e2e-test.yml` の `from_build: true` 経路) が通る
- 不正 JSON に対する回帰テストが追加されている (P2P モードで実 WebSocket クライアントから momo へ不正 JSON / キー欠落 / 型不一致を注入し、プロセスが落ちず英語エラーログが出て受信が継続することを確認。モックやスタブは使わない。pytest に WebSocket クライアントライブラリ (websockets 等) の依存追加が必要。接続先は P2P の起動ポートで、E2E テストでは動的ポートを使用)。P2P の回帰テストは「プロセスが落ちず受信が継続する」挙動を代表確認する (Sora / Ayame 固有の継続経路 (`DoRead()` → `return`) と Sora 固有の非同期コールバック内 try/catch は P2P のテストでは到達しないため、コードレビューで確認する)

## 解決方法

- `SoraClient::OnRead` / `SoraClient::OnMessage`、`AyameClient::OnRead`、`P2PWebsocketSession::OnRead` で `boost::json` のパースおよび `at` / `as_*` を try/catch し、失敗時は英語でエラーをロギングしたうえでメッセージを無視して受信を継続するようにした
- `SoraClient::OnMessage` では `ZlibHelper::Uncompress` を JSON 用 try の外に置き、zlib 例外を偶然握り潰さないようにした
- P2P 向けに実 WebSocket で不正 JSON（パース失敗・型不一致・キー欠落）を注入し、プロセスが生存したまま `register` に `accept` が返る回帰テストを追加した
- CI の全プラットフォームビルドおよび build 経由の E2E（`from_build: true`）が通過したことを確認した
- PR: https://github.com/shiguredo/momo/pull/464
