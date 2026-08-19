# WebSocket のプロキシ CONNECT レスポンスの HTTP ステータスを検証していない

- Created: 2026-08-19
- Completed: {YYYY-MM-DD}
- Branch: feature/fix-proxy-connect-status
- Polished: {YYYY-MM-DD}

## 目的

`Websocket` が HTTP プロキシ経由で WSS 接続する際、`CONNECT` レスポンスの HTTP ステータスを検証していない。プロキシが 407 (Proxy Authentication Required) や 403 を返しても `ec` は成功扱いになり、そのまま TLS ハンドシェイクを開始する。プロキシ認証失敗が検出できず、エラーが後段の TLS 失敗まで表面化しない。これを修正する。

## 現状

- `src/websocket.cpp` の `OnReadProxy()` (359-384 行) が `proxy_resp_` の `result()` を確認せず `wss_.reset(...)` して TLS ハンドシェイクへ進む
- `proxy_buffer_` に残った余剰バイト (200 応答後にプロキシが送るデータ) が SSL ストリームへ引き継がれない

## 設計方針

- `proxy_resp_.result()` が 2xx であることを確認し、非 2xx の場合は `on_connect` にエラーを通知する
- エラーメッセージにステータスコードと理由句を含める
- 余剰バイトの扱いを確認し、必要なら SSL ストリームへ引き継ぐ

## 完了条件

- プロキシが 407 / 403 を返した場合に適切なエラーが通知される
- 正常なプロキシ接続は従来通り動作する

## 解決方法

未着手 (PR 作成後に追記する)
