# Momo を使ってみる

## Raspberry Pi 向けのバイナリは以下にて提供しています

https://github.com/shiguredo/momo/releases にてバイナリをダウンロードしてください。

必要なライブラリをインストールしてご利用ください。

### それ以外のバイナリは自前でビルドしていただく必要があります。

[BUILD.md](doc/BUILD.md) をお読みください。

## ダウンロードしたパッケージ、解凍後の構成

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

## 注意

### 4K を利用する場合

- 4K を利用可能なのは現時点で x86_64 のみです
    - arm 系でも指定はできるようになっていますが、マシンリソースが足らず動作しません
- ロジクールの BRIO 4K のみ動作確認しています

### プレビュー版の機能

下記はプレビュー版の機能です

- --fixed-resolution          固定解像度

## 利用方法

Momo はモードを 3 つ持っています。

### P2P モード

Momo 自体がシグナリングサーバの機能も持つモードです。

### Ayame モード

オープンソースのシグナリングサーバ [WebRTC Signaling Server Ayame](https://github.com/OpenAyame/ayame) を利用するモードです。

### Sora モード

商用 WebRTC SFU の [WebRTC SFU Sora](https://sora.shiguredo.jp/) を利用するモードです。

### バージョン確認

```shell
$ ./momo --version
WebRTC Native Client Momo version 19.07.0
```

### P2P で動作を確認する

```shell
$ ./momo --no-audio --port 8080 p2p
```

http://[momo の IP アドレス]:8080/html/p2p.html にアクセスしてください。

### WebRTC Signaling Server Ayame で動作を確認する

```shell
$ ./momo --no-audio ayame wss://example.com/ws open-momo ayame-client-ud
```

### WebRTC SFU Sora で動作を確認する

**この機能を利用する場合は WebRTC SFU Sora のライセンス契約が必要です**

```shell
$ ./momo --no-audio sora --auto --video-codec VP8 --video-bitrate 500 wss://example.com/signaling open-momo
```

### コマンド

```
$ ./momo --version
WebRTC Native Client Momo 19.07.0
```

```
$ ./momo --help
Momo - WebRTC ネイティブクライアント
Usage: ./momo [OPTIONS] [SUBCOMMAND]

Options:
  -h,--help                   Print this help message and exit
  --no-video                  ビデオを表示しない
  --no-audio                  オーディオを出さない
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
  ayame                       WebRTC Signaling Server Ayame
  sora                        WebRTC SFU Sora
```

#### p2p


```
$ ./momo p2p --help
P2P
Usage: ./momo p2p [OPTIONS]

Options:
  -h,--help                   Print this help message and exit
  --document-root Directory   配信ディレクトリ
```


#### ayame


 ```
$ ./momo ayame --help
WebRTC Signaling Server Ayame
Usage: ./momo ayame [OPTIONS] SIGNALING-URL ROOM-ID CLIENT-ID
 Positionals:
  SIGNALING-URL TEXT REQUIRED シグナリングホスト
  ROOM-ID TEXT REQUIRED       ルーム ID
  CLIENT-ID TEXT REQUIRED     クライアント ID
 Options:
  -k,--key                    キー
  -h,--help                   Print this help message and exit
```

#### sora

```
$ ./momo sora --help
WebRTC SFU Sora
Usage: ./momo sora [OPTIONS] SIGNALING-URL CHANNEL-ID

Positionals:
  SIGNALING-URL TEXT REQUIRED シグナリングホスト
  CHANNEL-ID TEXT REQUIRED    チャンネルID

Options:
  --video-codec STR in [VP8,VP9,H264]
                              ビデオコーデック
  --audio-codec STR in [OPUS,PCMU]
                              オーディオコーデック
  --video-bitrate INT in [1 - 30000]
                              ビデオのビットレート
  --audio-bitrate INT in [6 - 510]
                              オーディオのビットレート
  -h,--help                   Print this help message and exit
  --auto                      自動接続する
```

## うまく動作しない時

- カメラを uv4l など他のプロセスが利用していないか確認してください

## Q&A

### コーデックの指定やビットレートを利用したい

指定は WebRTC SFU Sora を利用したときだけ可能です。ただし受信側の SDP を動的に書き換えることで対応可能です。
