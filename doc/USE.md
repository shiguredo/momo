# Momo を使ってみる

**macOS 版と Windows 版のバイナリ提供はしていません、自力でのビルドをお願いします**

https://github.com/shiguredo/momo/releases にて利用したい環境のバイナリをダウンロードしてください。

必要なライブラリをインストールしてご利用ください。

## 解凍後の構成

```
$ tree
.
├── html
│   ├── p2p.html
│   └── webrtc.js
├── LICENSE
├── momo
└── NOTICE
```

## Raspbian で Raspberry Pi を利用する場合

Raspbian にて下記を実行してください。

```
$ sudo apt-get install libnspr4 libnss3
```

## Raspbian で Raspberry Pi の Raspberry Pi 用カメラを利用する場合

これは USB カメラを利用する場合は不要なオプションです。

raspi-config で Camera を Enable にしてください。

さらに、以下のコマンドか

```
$ sudo modprobe bcm2835-v4l2
```

/etc/modules に

```
bcm2835-v4l2
```

を追加して再起動してください。

## Ubuntu 16.04 で ARMv8 対応のバイナリを利用する場合

下記を実行してください。

```
$ sudo apt-get install libx11-dev libnss3 libxext6
```

上手く動かない場合は下記のライブラリもインストールしてみてください。

```
$ sudo apt-get install libx11-6 libxau6 libxdmcp6 libxcb1 libnspr4 libexpat1 libasound2
```

## 注意

### 4K を利用する場合

- 4K を利用可能なのは現時点で x86_64 のみです
    - arm 系でも指定はできるようになっていますが、マシンリソースが足らず動作しません
- ロジクールの BRIO 4K 動作確認しています

### プレビュー版の機能

下記はプレビュー版の機能です

- --fixed-resolution          固定解像度

## 利用方法

Momo はモードを ２ つ持っています。一つが P2P モードで Momo 自体がシグナリングサーバの機能も持つモードです。

もう一つは WebRTC SFU Sora と接続する方法です。

### バージョン確認

```shell
$ momo --version
WebRTC Native Client Momo version 19.02.0
```

### P2P で動作を確認する

```shell
$ momo --no-audio --video-codec VP8 --video-bitrate 800 --port 8080 p2p
```

http://[momo の IP アドレス]:8080/html/p2p.html にアクセスしてください。

### WebRTC SFU Sora で動作を確認する

**この機能を利用する場合は WebRTC SFU Sora のライセンス契約が必要です**

```shell
$ momo --no-audio --video-codec VP8 --video-bitrate 500 \
       sora --auto wss://example.com/signaing open-momo
```

### コマンド

```
$ ./momo --version
WebRTC Native Client Momo 19.02.0
```

```
$ ./momo --help
Momo - WebRTC ネイティブクライアント
Usage: ./momo [OPTIONS] [SUBCOMMAND]

Options:
  -h,--help                   Print this help message and exit
  --no-video                  ビデオを表示しない
  --no-audio                  オーディオを出さない
  --video-codec STR in [VP8,VP9,H264]
                              ビデオコーデック
  --audio-codec STR in [OPUS,PCMU]
                              オーディオコーデック
  --video-bitrate INT in [1 - 30000]
                              ビデオのビットレート
  --audio-bitrate INT in [6 - 510]
                              オーディオのビットレート
  --resolution STR in [QVGA,VGA,HD,FHD,4K]
                              解像度
  --framerate INT in [1 - 60] フレームレート
  --fixed-resolution          固定解像度
  --priority STR in [BALANCE,FRAMERATE,RESOLUTION]
                              優先設定 (Experimental)
  --port INT in [0 - 65535]   ポート番号
  --daemon                    デーモン化する
  --version                   バージョン情報の表示
  --log-level INT in [0 - 5]  ログレベル

Subcommands:
  p2p                         P2P
  sora                        WebRTC SFU Sora
```


```
$ ./momo p2p --help
P2P
Usage: ./momo p2p [OPTIONS]

Options:
  -h,--help                   Print this help message and exit
  --document-root Directory   配信ディレクトリ
```


```
$ ./momo --no-audio --video-codec VP8 --video-bitrate 1500 \
         p2p --port 8080
```


```
$ ./momo sora --help
WebRTC SFU Sora
Usage: ./momo sora [OPTIONS] SIGNALING-URL CHANNEL-ID

Positionals:
  SIGNALING-URL TEXT REQUIRED シグナリングホスト
  CHANNEL-ID TEXT REQUIRED    チャンネルID

Options:
  -h,--help                   Print this help message and exit
  --auto                      自動接続する
```

```
$ ./momo --no-audio --video-codec VP8 --video-bitrate 500 \
         sora --auto wss://example.com/signaing open-momo
```

