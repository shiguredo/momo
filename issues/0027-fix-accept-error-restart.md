# P2P / metrics サーバが accept エラーでサービスを永久停止する

- Created: 2026-08-19
- Completed: {YYYY-MM-DD}
- Branch: feature/fix-accept-error-restart
- Polished: 2026-08-20
- Milestone: 2026.1.0

## 目的

`P2PServer` と `MetricsServer` の `OnAccept` が accept エラー時に `DoAccept()` を再開せず return するため、EMFILE 等の一時エラーが一度発生すると以降の接続受付が完全に停止する。特に P2P サーバは `0.0.0.0:8080` に公開されており、一時エラーでサービスが無応答になる。`SoraServer` はエラー時も `DoAccept()` を再開するのに整合性が取れていない。これを修正する。

## 現状

- `src/p2p/p2p_server.cpp` の `P2PServer::OnAccept()`: エラー時に `MOMO_BOOST_ERROR` を出力して return
- `src/metrics/metrics_server.cpp` の `MetricsServer::OnAccept()`: 同様
- `src/sora/sora_server.cpp` の `SoraServer::OnAccept()`: エラーでも `DoAccept()` を再開する

## 設計方針

- 両サーバとも、エラー時も `DoAccept()` を再開する (SoraServer と同様。ただし EMFILE 等が続く間は再開が連続してビジーループになりうるため、再開前に遅延を挟むかは実装時に判断する)
- `acceptor_.is_open()` が false (リスナーが閉じられた) 場合は再開しない。このガードは SoraServer にはない追加の防御であり、現状のアーキテクチャでは acceptor は明示的に閉じられないため防衛的なものである

## 完了条件

- accept エラー後もサービスが接続受付を継続する (`ulimit -n` で fd 数を制限し、fd 上限まで接続を張って EMFILE を発生させ、fd 解放後に再接続できることを手動確認する)
- 通常の接続フローは従来通り動作する

## 解決方法

未着手 (PR 作成後に追記する)
