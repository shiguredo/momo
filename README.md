# WebRTC Native Client Momo (Rust)

[![GitHub tag (latest SemVer)](https://img.shields.io/github/tag/shiguredo/momo-rs.svg)](https://github.com/shiguredo/momo-rs)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

## About Shiguredo's open source software

We will not respond to PRs or issues that have not been discussed on Discord. Also, Discord is only available in Japanese.

Please read <https://github.com/shiguredo/oss> before use.

## 時雨堂のオープンソースソフトウェアについて

利用前に <https://github.com/shiguredo/oss> をお読みください。

## 概要

WebRTC Native Client Momo の Rust 実装です。libwebrtc を利用し、ブラウザなしで動作する WebRTC ネイティブクライアントです。

## モード

### P2P モード

Momo 自体がシグナリングサーバーと HTTP サーバーを持っているため、完全な P2P モードでの利用ができます。
ブラウザから Momo にアクセスするだけで利用できます。

### Ayame モード

WebRTC シグナリングサーバー [Ayame](https://github.com/OpenAyame/ayame) に対応したモードです。

### Sora モード

WebRTC SFU [Sora](https://sora.shiguredo.jp) に対応したモードです。

## 動作環境

- macOS 15 arm64
- macOS 14 arm64
- Ubuntu 24.04 x86_64
- Ubuntu 22.04 x86_64

## ビルド

Rust 1.88 以上が必要です。

```bash
cargo build --release
```

## クロスコンパイル

Dev Container (Debian Trixie) 内でクロスコンパイルを行います。

### 前提条件

- Docker
- VS Code + Dev Containers 拡張

### 対応ターゲット

| ターゲット | sysroot 生成 | ビルド |
|---|---|---|
| Raspberry Pi (arm64) | `make sysroot-raspberry-pi` | `make sysroot-build-raspberry-pi` |
| Ubuntu 24.04 (arm64) | `make sysroot-ubuntu-24.04_arm64` | `make sysroot-build-ubuntu-24.04_arm64` |
| Ubuntu 22.04 (arm64) | `make sysroot-ubuntu-22.04_arm64` | `make sysroot-build-ubuntu-22.04_arm64` |

リリースビルドは各ターゲットに `-release` を付けてください（例: `make sysroot-build-raspberry-pi-release`）。

### 手順

1. VS Code でリポジトリを開き、コマンドパレットから **Dev Containers: Reopen in Container** を選択する
2. sysroot を生成する（初回のみ）

```bash
make sysroot-raspberry-pi
```

3. ビルドする

```bash
make sysroot-build-raspberry-pi
```

4. macOS に戻る場合はコマンドパレットから **Dev Containers: Reopen Folder Locally** を選択する

## 使い方

### デバイス一覧の取得

利用可能な映像入力・音声入力デバイスを JSON 形式で表示します。

```bash
./momo --list-devices
```

### P2P モード

```bash
./momo p2p --port 8080 --document-root html
```

ブラウザで `http://localhost:8080` にアクセスしてください。

**オプション:**

- `--port PORT`: リッスンポート（デフォルト: 8080）
- `--document-root PATH`: HTTP ドキュメントルートディレクトリ（デフォルト: html）

### Ayame モード

```bash
./momo ayame \
  --signaling-url wss://example.com/signaling \
  --room-id your-room-id
```

**オプション:**

- `--signaling-url URL`: シグナリング URL
- `--room-id ID`: ルーム ID
- `--client-id ID`: クライアント ID（任意）
- `--signaling-key KEY`: シグナリングキー（任意）
- `--direction DIRECTION`: 送受信方向（sendrecv / sendonly / recvonly、デフォルト: sendrecv）

### Sora モード

```bash
./momo sora \
  --signaling-urls wss://example.com/signaling \
  --channel-id your-channel-id \
  --role sendonly
```

**オプション:**

- `--signaling-urls URLS`: シグナリング URL（カンマ区切りで複数指定可）
- `--channel-id ID`: チャネル ID
- `--role ROLE`: ロール（sendonly / recvonly / sendrecv、デフォルト: sendonly）
- `--video BOOL`: 映像送信（デフォルト: true）
- `--audio BOOL`: 音声送信（デフォルト: true）
- `--video-codec-type TYPE`: 映像コーデック（VP8 / VP9 / AV1 / H264 / H265）
- `--audio-codec-type TYPE`: 音声コーデック（OPUS）
- `--video-bit-rate RATE`: 映像ビットレート（デフォルト: 0）
- `--audio-bit-rate RATE`: 音声ビットレート（デフォルト: 0）
- `--spotlight BOOL`: スポットライト（デフォルト: false）
- `--simulcast BOOL`: サイマルキャスト（デフォルト: false）
- `--metadata JSON`: connect メッセージに含めるメタデータ

### プレビュー機能

sendonly 時にキャプチャ映像を SDL3 ウィンドウでリアルタイム表示します。

**ビルド:**

```bash
cargo build --release --features sora,preview
```

**使い方:**

```bash
# フェイク映像でプレビュー
./momo --use-raw-player --fake-capture-device sora \
  --signaling-urls wss://example.com/signaling \
  --channel-id your-channel-id \
  --role sendonly

# 実デバイスでプレビュー
./momo --use-raw-player sora \
  --signaling-urls wss://example.com/signaling \
  --channel-id your-channel-id \
  --role sendonly
```

**プレビュー関連オプション:**

- `--use-raw-player`: プレビューウィンドウを表示する（`preview` feature が必要）
- `--window-width WIDTH`: プレビューウィンドウ幅（デフォルト: 640）
- `--window-height HEIGHT`: プレビューウィンドウ高さ（デフォルト: 480）

プレビューウィンドウを閉じても Sora 接続は継続します。

### グローバルオプション

| オプション | 説明 |
|---|---|
| `--no-video-input-device` | 映像入力デバイスを使用しない |
| `--no-audio-device` | 音声デバイスを使用しない |
| `--fake-capture-device` | フェイク映像/音声キャプチャデバイスを使用する |
| `--no-google-stun` | Google STUN サーバーを使用しない |
| `--video-input-device DEVICE` | 映像デバイスを名前またはインデックスで指定 |
| `--audio-input-device DEVICE` | 音声入力デバイスを名前またはインデックスで指定 |
| `--list-devices` | 利用可能なデバイス一覧を JSON 形式で出力して終了 |

## momo (C++ 版) との違い

詳細は [docs/MOMO.md](docs/MOMO.md) を参照してください。

### momo-rs 独自機能

| 機能 | 説明 |
|---|---|
| `--cacert` | CA 証明書ファイルを PEM 形式で指定。momo にはないオプション |

### 仕様の違い

| 機能 | momo | momo-rs |
|---|---|---|
| `--libcamera-control` の形式 | `KEY VALUE` (2 引数) | `KEY=VALUE` (1 引数) |
| プロキシ対応 | 全モード (HTTP CONNECT) | Sora モードのみ (sora_sdk ProxyInfo)。P2P / Ayame は非対応 |
| TLS 証明書検証 | OpenSSL | rustls + OS ネイティブ証明書ストア (rustls_platform_verifier) |
| フェイクキャプチャ | WebRTC FakeVideoTrackSource | raden によるアニメーションフレーム生成 |
| ログ出力 | Boost.Beast + FileRotatingLogSink | tracing_subscriber (stdout のみ) |
| WebRTC 実装 | libwebrtc (C++) を直接使用 | shiguredo_webrtc (Rust バインディング) 経由で使用 |
| OpenH264 エンコーダー優先順位 | HW > OpenH264 > ビルトイン | `--use-v4l2-encoder` (HW) > `--openh264` (SW) > ビルトイン |

### 未実装機能

自動再接続:

- Sora モード自動再接続 (sora_sdk 内部の対応が必要)
- Ayame モード自動再接続 (momo は watchdog 機構付き)

音声処理 (shiguredo_webrtc API 不足のため pending):

- `--audio-output-device`
- `--disable-echo-cancellation` / `--disable-auto-gain-control` / `--disable-noise-suppression` / `--disable-highpass-filter`

コーデック選択:

- `--{codec}-encoder` / `--{codec}-decoder` (HW バックエンド未実装のため pending)
- `--video-codec-engines`
- `--hw-mjpeg-decoder`

HW エンコード:

- Jetson (H.264/H.265)
- NVIDIA NvCodec/CUDA (H.264/H.265)
- Intel oneVPL (H.264/H.265)

その他:

- スクリーンキャプチャ (`--screen-capture`)
- ログファイル出力 (ローテーション)
- SoraServer (`--port` / `--auto` による Sora モードの HTTP サーバー)
- `--data-channel-signaling-timeout` (sora_sdk に API なし)

## ライセンス

Apache License 2.0

```text
Copyright 2026-2026, Shiguredo Inc.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
```
