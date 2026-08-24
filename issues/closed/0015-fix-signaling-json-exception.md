# シグナリングサーバから受信した JSON のパース例外未処理でプロセスが落ちる

- Created: 2026-08-19
- Completed: 2026-08-24
- Branch: feature/fix-signaling-json-exception
- Polished: 2026-08-19
- Milestone: 2026.1.0

## 目的

Sora / Ayame / P2P の各シグナリングでサーバから受信する JSON メッセージを `boost::json` でパースする際、`parse()` / `at()` / `as_bool()` / `as_string()` が投げる `boost::system::system_error` を catch していない。サーバが壊れた・悪意のある JSON を送ると例外が `io_context::run()` から伝播し `std::terminate` でプロセスが即死する。Sora / Ayame はシグナリングサーバからの攻撃、P2P は `src/main.cpp` が `0.0.0.0:8080` (`--p2p-port` デフォルト `8080`) に認証なしで bind するため任意のネットワークユーザーからの DoS が可能。これを修正する。

## 現状

- `src/sora/sora_client.cpp` の `SoraClient::OnRead()` が `boost::json::parse()` と `json_message.at()` / `as_string()` / `as_bool()` / `as_array()` を try/catch なしで呼び出している (`CreateRTCConnection` の `jconfig.at("iceServers")` 等を含む)。`SoraClient::OnMessage()` の DataChannel 経由は `parse(data, ec)` でパースエラーはハンドリングしているが、`json.at("type")` 以降のキーアクセスは例外未処理のまま
- `src/p2p/p2p_websocket_session.cpp` の `P2PWebsocketSession::OnRead()` が `recv_message.at("type")` / `at("sdp")` / `ice.at()` 等を未処理で使用。`parse(recv_string, jec)` の `error_code` オーバーロードでパース失敗は握り潰すが `at()` は裸のまま
- `src/ayame/ayame_client.cpp` の `AyameClient::OnRead()` が同様に `parse()` / `at()` を try/catch なしで使用
- sora-cpp-sdk 側の `SoraClient` も同様の構造を持つが、momo 側の `src/sora/sora_client.cpp` は vendoring 後に独自の DataChannel 圧縮・watchdog 処理が追加されており、momo 側で個別に修正が必要
- `src/sora/sora_session.cpp` の `SoraSession::OnRead()` の `/mute` ハンドラも `recv_json.at("audio").as_bool()` / `at("video")` を try/catch なしで呼ぶが、`SoraSession` は `127.0.0.1` bind のため本 issue ではスコープ外とし、必要なら別途対応する

## 設計方針

- シグナリング JSON のパース・アクセスを `try` / `catch (const boost::system::system_error&)` / `catch (const std::exception&)` で包み、パース失敗・キー欠落・型不一致の場合はエラーを英語でログ出力してそのメッセージを無視する。`catch` 後は必ず `DoRead()` を再開し (P2P の `Guard` パターンまたは `finally` 的に `DoRead()` を呼ぶ)、以降の受信が停止しないようにする。`boost::json::parse()` の `error_code` オーバーロードを使う箇所も、`at()` / `as_*()` の例外は同様に catch する
- 異常メッセージで接続フローが進まない場合は、既存の `Watchdog` のタイムアウト (Sora 30/60 秒、Ayame 30/60 秒、P2P 30 秒) で再接続に至る。watchdog 自体の挙動は変えず、ログ出力で原因追跡可能にする
- 3 クライアント (Sora / P2P / Ayame) の `OnRead` / `OnMessage` の先頭で try/catch を追加してクラッシュを止めることを最優先とする。共通の安全な JSON アクセスヘルパー (`find()` と型チェックで例外を投げない経路) への置換は任意とし、まずは try/catch で堅牢化する
- sora-cpp-sdk 側の修正を参考にするが、momo 側の `src/sora-cpp-sdk/` は `0013-update-sora-cpp-sdk-sdl3-cli11` で同期されるため、本 issue では `src/sora/` / `src/ayame/` / `src/p2p/` の momo 固有コードを対象とする。`0029-fix-zlib-uncompress-memory` の `ZlibHelper::Uncompress` 例外とは catch のスコープを分け、JSON 用 catch が zlib 例外を偶然握り潰さないようにする

## 完了条件

- 不正な JSON / キー欠落 / 型不一致メッセージを送ってもプロセスが落ちず、英語でエラーログが出力され `DoRead()` が継続する
- 正常なシグナリングフロー (Sora / Ayame / P2P の offer / answer / candidate / ping / switched) が従来通り動作する
- 各モードの E2E テスト (`e2e-test.yml` の `from_build: true` 経路) が通る
- 不正 JSON に対する回帰テストが追加されている (単体または結合、モックやスタブは使わず実 WebSocket での注入または `SoraClient::OnRead` 直接テスト)

## 解決方法

- `SoraClient::OnRead` / `SoraClient::OnMessage`、`AyameClient::OnRead`、`P2PWebsocketSession::OnRead` で `boost::json` のパースおよび `at` / `as_*` を try/catch し、失敗時は英語でエラーをロギングしたうえでメッセージを無視して受信を継続するようにした
- `SoraClient::OnMessage` では `ZlibHelper::Uncompress` を JSON 用 try の外に置き、zlib 例外を偶然握り潰さないようにした
- P2P 向けに実 WebSocket で不正 JSON（パース失敗・型不一致・キー欠落）を注入し、プロセスが生存したまま `register` に `accept` が返る回帰テストを追加した
- CI の全プラットフォームビルドおよび build 経由の E2E（`from_build: true`）が通過したことを確認した
- PR: https://github.com/shiguredo/momo/pull/464
