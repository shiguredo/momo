# Raspberry Pi で Momo を使ってみる

## Raspberry Pi 向けのバイナリは以下にて提供しています

https://github.com/shiguredo/momo/releases にて最新版のバイナリをダウンロードしてください。

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

## 準備

### パッケージのインストール

下記を実行してください。

```
$ sudo apt-get install libnspr4 libnss3
```

### Raspbian で Raspberry Pi の Raspberry Pi 用カメラを利用する場合

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

## まずは動かしてみる

Momo 自体がシグナリングサーバの機能を持つ test モードを利用して動かしてみてください。

```shell
$ ./momo --no-audio --port 8080 test
```

http://[momo の IP アドレス]:8080/html/test.html にアクセスして接続してみてください。

うまく接続できたら、次は是非 Ayame を利用して動かしてみてください。

Ayame を利用する場合は [USE_AYAME.md](doc/USE_AYAME.md) をご確認ください。

## Raspberry Pi 向けビルド向けの追加のオプション

### --use-native

`--use-native` は ハードウェアによるビデオのリサイズ と USB カメラ用の場合 MJPEG をハードウェアデコードします。

```shell
$ ./momo --use-native --no-audio --port 8080 test
```

### --force-i420

`--force-i420` は Raspberry Pi 専用カメラ用では MJPEG を使うとパフォーマンスが落ちるため HD 以上の解像度でも MJPEG にせず強制的に I420 でキャプチャーします。
USBカメラでは逆にフレームレートが落ちるため使わないでください。


```shell
$ ./momo --force-i420 --no-audio --port 8080 test
```

## Raspberry Pi 専用カメラでパフォーマンスが出ない

[Raspbian で Raspberry Pi の Raspberry Pi 用カメラを利用する場合](#raspbian-で-raspberry-pi-の-raspberry-pi-用カメラを利用する場合)通りに設定されているか確認してください。特に `max_video_width=2592 max_video_height=1944` が記載されていなければ高解像度時にフレームレートが出ません。

Raspberry Pi 専用カメラ利用時には `--use-native --force-i420` オプションを併用するとCPU使用率が下がりフレームレートが上がります。例えば、 RaspberryPi Zero の場合には

```shell
$ ./momo --resolution=HD --framerate=20 --force-i420 --use-native test
```

がリアルタイムでの最高解像度設定となります。パフォーマンスが限られた Zero でリアルタイムにするには framerate を制限することも重要になります。

## Raspberry Pi で USB カメラ利用時に use-native を使ってもフレームレートが出ない

USB カメラ利用時には `--use-native` を使わない方がフレームレートは出ます。しかし `--use-native` を使ってCPU使用率を下げた状態で利用したい場合は /boot/config.txt の末尾に下記を追記してください

```
gpu_mem=128
force_turbo=1
avoid_warnings=2
```

この設定であれば HD は 30fps, FHD では 15fps 程度の性能を発揮します。


## Sora モードで確認する

商用 WebRTC SFU の [WebRTC SFU Sora](https://sora.shiguredo.jp/) を利用するモードです。

**この機能を利用する場合は WebRTC SFU Sora のライセンス契約が必要です**

```shell
$ ./momo --no-audio sora --auto --video-codec VP8 --video-bitrate 500 wss://example.com/signaling open-momo
```
