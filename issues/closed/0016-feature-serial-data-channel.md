# シリアルデータチャネルの実装

Created: 2026-03-24
Completed: 2026-03-24
Model: Opus 4.6

## 概要

`--serial DEVICE,BAUDRATE` オプションで指定されたシリアルポートを開き、ブラウザが作成した "serial" ラベルの DataChannel と双方向にデータを中継する機能。

## 解決方法

- `src/serial.rs` を新規作成
- シリアルポートの開設: `std::fs::OpenOptions` + `libc::tcsetattr` / `cfmakeraw` / `cfsetspeed`
- 非同期化: `libc::fcntl` で `O_NONBLOCK` 設定後、`tokio::io::unix::AsyncFd` でラップ
- P2P / Ayame モードの `PeerConnectionObserverHandler::on_data_channel` で "serial" ラベルの DataChannel を受け取り、シリアルブリッジを起動
- 外部依存なし (tokio-serial 等を使わず libc + tokio AsyncFd で実装)
- Linux 限定 (`#[cfg(target_os = "linux")]`)
