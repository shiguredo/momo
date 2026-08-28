# WebSocket のプロキシ CONNECT レスポンスの HTTP ステータスを検証していない

- Created: 2026-08-19
- Completed: 2026-08-28
- Branch: feature/fix-proxy-connect-status
- Polished: 2026-08-20
- Milestone: 2026.1.0

## 目的

`Websocket` が HTTP プロキシ経由で WSS 接続する際、`CONNECT` レスポンスの HTTP ステータスを検証していない。プロキシが 407 (Proxy Authentication Required) や 403 を返しても `ec` は成功扱いになり、そのまま TLS ハンドシェイクを開始する。プロキシ認証失敗が検出できず、エラーが後段の TLS 失敗まで表面化しない。これを修正する。

## 現状

- `src/websocket.cpp` の `OnReadProxy()` が `proxy_resp_` の `result()` を確認せず `wss_.reset(...)` して TLS ハンドシェイクへ進む
- `proxy_buffer_` に残った余剰バイトの SSL ストリームへの引継ぎは本 issue の対象外とする (CONNECT 成功後のプロキシはトンネルとして中継するだけで、TLS では接続先サーバが ClientHello 受信前にデータを送ることはなく、実質発生しない)
- 同一関数を変更する 0014 と並行しており、0014 を先にマージしてから本 issue を rebase する

## 設計方針

- `proxy_resp_.result()` が 2xx であることを確認し、非 2xx の場合は `on_connect` にエラーを通知する
- ステータスコードと理由句はエラーログに出力する (`on_connect` は `error_code` しか運べないため)

## 完了条件

- プロキシが 407 / 403 を返した場合にエラーが通知される
- 正常なプロキシ接続は従来通り動作する

## 解決方法

`OnReadProxy` で CONNECT 応答を `proxy_resp_parser_->get()` から取り、2xx でなければ TLS に進まない。ログにステータスと理由句を出し、`on_connect` には `permission_denied` を渡す。非 2xx の HTTP サーバで `Proxy CONNECT failed: 501 Not Implemented` を確認し、TLS に進まないことを確認した。CI のビルドと E2E は成功。PR は https://github.com/shiguredo/momo/pull/471 。
