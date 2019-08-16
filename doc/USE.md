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
│   ├── test.html
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
$ sudo modprobe bcm2835-v4l2 max_video_width=2592 max_video_height=1944
```

/etc/modules の末尾に

```
bcm2835-v4l2 max_video_width=2592 max_video_height=1944
```

を追加して再起動してください。

## 注意

### 4K を利用する場合

- 4K を利用可能なのは現時点で x86_64 のみです
    - arm 系でも指定はできるようになっていますが、マシンリソースが足らず動作しません
- ロジクールの BRIO 4K のみ動作確認しています

### プレビュー版の機能

下記はプレビュー版の機能です

- 固定解像度 --fixed-resolution

## 利用方法

Momo はモードを 3 つ持っています。

### Test モード

Momo 自体がシグナリングサーバの機能も持つ開発モードです。

### Ayame モード

オープンソースのシグナリングサーバ [WebRTC Signaling Server Ayame](https://github.com/OpenAyame/ayame) を利用するモードです。

### Sora モード

商用 WebRTC SFU の [WebRTC SFU Sora](https://sora.shiguredo.jp/) を利用するモードです。

### バージョン確認

```shell
$ ./momo --version
WebRTC Native Client Momo version 19.08.0
```

### 開発モードで動作を確認する

```shell
$ ./momo --no-audio --port 8080 test
```

http://[momo の IP アドレス]:8080/html/test.html にアクセスしてください。

### WebRTC Signaling Server Ayame で動作を確認する

```shell
$ ./momo --no-audio ayame wss://example.com/ws open-momo
```

### WebRTC SFU Sora で動作を確認する

**この機能を利用する場合は WebRTC SFU Sora のライセンス契約が必要です**

```shell
$ ./momo --no-audio sora --auto --video-codec VP8 --video-bitrate 500 wss://example.com/signaling open-momo
```

### コマンド

```
$ ./momo --version
WebRTC Native Client Momo 19.08.0
```

```
$ ./momo --help
Momo - WebRTC ネイティブクライアント
Usage: ./momo [OPTIONS] [SUBCOMMAND]

Options:
  -h,--help                   Print this help message and exit
  --no-video                  ビデオを表示しない
  --no-audio                  オーディオを出さない
  --resolution TEXT:{4K,FHD,HD,QVGA,VGA}
                              解像度
  --framerate INT:INT in [1 - 60]
                              フレームレート
  --fixed-resolution          固定解像度
  --priority TEXT:{BALANCE,FRAMERATE,RESOLUTION}
                              優先設定 (Experimental)
  --port INT:INT in [0 - 65535]
                              ポート番号(デフォルト:8080)
  --daemon                    デーモン化する
  --version                   バージョン情報の表示
  --log-level INT:value in {verbose->0,info->1,warning->2,error->3,none->4} OR {0,1,2,3,4}
                              ログレベル

Subcommands:
  test                        開発向け
  ayame                       WebRTC Signaling Server Ayame
  sora                        WebRTC SFU Sora
```

#### Raspberry Pi 向けビルドのみ追加のオプションが存在します

- --use-native は ハードウェアによるビデオのリサイズ と USB カメラ用の場合 MJPEG をハードウェアデコードします。
- --force-i420 は Raspberry Pi 専用カメラ用では MJPEG を使うとパフォーマンスが落ちるため HD 以上の解像度でも MJPEG にせず強制的に I420 でキャプチャーします。USBカメラでは逆にフレームレートが落ちるため使わないでください。

```
$ ./momo --help
Momo - WebRTC ネイティブクライアント
Usage: ./momo [OPTIONS] [SUBCOMMAND]

Options:
  -h,--help                   Print this help message and exit
  --no-video                  ビデオを表示しない
  --no-audio                  オーディオを出さない
  --force-i420                強制的にI420にする
  --use-native                MJPEGのデコードとビデオのリサイズをハードウェアで行う
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
  test                        開発向け
  ayame                       WebRTC Signaling Server Ayame
  sora                        WebRTC SFU Sora

```

#### test


```
$ ./momo test --help
開発向け
Usage: ./momo test [OPTIONS]

Options:
  -h,--help                   Print this help message and exit
  --document-root TEXT:DIR    配信ディレクトリ
```

#### ayame


 ```
$ ./momo ayame --help
WebRTC Signaling Server Ayame
Usage: ./momo ayame [OPTIONS] SIGNALING-URL ROOM-ID

Positionals:
  SIGNALING-URL TEXT REQUIRED シグナリングホスト
  ROOM-ID TEXT REQUIRED       ルームID

Options:
  -h,--help                   Print this help message and exit
  --client-id TEXT            クライアントID
  --signaling-key TEXT        シグナリングキー
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
  -h,--help                   Print this help message and exit
  --auto                      自動接続する
  --video-codec TEXT:{H264,VP8,VP9}
                              ビデオコーデック
  --audio-codec TEXT:{OPUS,PCMU}
                              オーディオコーデック
  --video-bitrate INT:INT in [1 - 30000]
                              ビデオのビットレート
  --audio-bitrate INT:INT in [6 - 510]
                              オーディオのビットレート
```

## うまく動作しない時

- カメラを uv4l など他のプロセスが利用していないか確認してください

## Q&A

### コーデックの指定やビットレートを利用したい

指定は WebRTC SFU Sora を利用したときだけ可能です。ただし受信側の SDP を動的に書き換えることで対応可能です。

### Raspberry Pi 専用カメラでパフォーマンスが出ない

[Raspbian で Raspberry Pi の Raspberry Pi 用カメラを利用する場合](#raspbian-で-raspberry-pi-の-raspberry-pi-用カメラを利用する場合)通りに設定されているか確認してください。特に `max_video_width=2592 max_video_height=1944` が記載されていなければ高解像度時にフレームレートが出ません。

Raspberry Pi 専用カメラ利用時には `--use-native --force-i420` オプションを併用するとCPU使用率が下がりフレームレートが上がります。例えば、 RaspberryPi Zero の場合には

```shell
$ ./momo --resolution=HD --framerate=20 --force-i420 --use-native test
```

がリアルタイムでの最高解像度設定となります。パフォーマンスが限られた Zero でリアルタイムにするには framerate を制限することも重要になります。

### Raspberry Pi で USB カメラ利用時に use-native を使ってもフレームレートが出ない

USB カメラ利用時には `--use-native` を使わない方がフレームレートは出ます。しかし `--use-native` を使ってCPU使用率を下げた状態で利用したい場合は /boot/config.txt の末尾に下記を追記してください

```
gpu_mem=128
force_turbo=1
avoid_warnings=2
```

この設定であれば HD は 30fps, FHD では 15fps 程度の性能を発揮します。
