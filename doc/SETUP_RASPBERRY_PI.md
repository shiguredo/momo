# Raspberry Pi (Raspbian) で Momo を使ってみる

## Raspberry Pi 向けのバイナリは以下にて提供しています

https://github.com/shiguredo/momo/releases にて最新版のバイナリをダウンロードしてください。

- Raspberry Pi 2 や 3 や 4 を利用する場合は、 `momo-<VERSION>_raspbian-buster_armv7.tar.gz` を利用してください
- Raspberry Pi Zero や 1 を利用する場合は、 `momo-<VERSION>_raspbian-buster_armv6.tar.gz` を利用してください

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
$ sudo apt-get install libnspr4 libnss3 libsdl2-dev
```

### Raspbian で Raspberry Pi の Raspberry Pi 用カメラを利用する場合

これは USB カメラを利用する場合は不要なオプションです。

raspi-config で Camera を Enable にしてください。

加えて、以下のコマンドを実行してください

```
$ sudo modprobe bcm2835-v4l2 max_video_width=2592 max_video_height=1944
```

## 使ってみる

[USE_TEST.md](USE_TEST.md) をご確認ください。

## ビデオデバイスの指定

ビデオデバイスの指定については [LINUX_VIDEO_DEVICE.md](LINUX_VIDEO_DEVICE.md) をご確認ください。

## Raspberry Pi 向けの追加のオプション

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

Raspberry Pi 専用カメラ利用時には `--use-native --force-i420` オプションを併用するとCPU使用率が下がりフレームレートが上がります。例えば、 Raspberry Pi Zero の場合には

```shell
$ ./momo --resolution=HD --force-i420 --use-native test
```

がリアルタイムでの最高解像度設定となります。

## Raspberry Pi で USB カメラ利用時に use-native を使ってもフレームレートが出ない

USB カメラ利用時には `--use-native` を使わない方がフレームレートは出ます。しかし `--use-native` を使ってCPU使用率を下げた状態で利用したい場合は /boot/config.txt の末尾に下記を追記してください

```
gpu_mem=128
force_turbo=1
avoid_warnings=2
```

この設定であれば HD は 30fps, FHD では 15fps 程度の性能を発揮します。
