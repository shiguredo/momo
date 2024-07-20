# Raspberry Pi (Raspberry-Pi-OS) で Momo を使ってみる

## 注意

Raspberry Pi OS のレガシー版には対応しておりません。最新版の Raspberry Pi OS を利用してください

## Raspberry Pi 向けのバイナリは以下にて提供しています

<https://github.com/shiguredo/momo/releases> にて最新版のバイナリをダウンロードしてください。

- Raspberry Pi OS 64 bit を利用する場合は、 `momo-<VERSION>_raspberry-pi-os_armv8.tar.gz` を利用してください

## ダウンロードしたパッケージ、解凍後の構成

```console
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

```bash
sudo apt-get update
sudo apt-get upgrade
sudo apt-get install libnspr4 libnss3
sudo apt-get install libcamera0
```

#### Raspberry Pi OS Lite を利用する場合

Raspberry Pi Lite では映像に関するパッケージが入っていないため、`ldd ./momo | grep not` を実行し、不足しているパッケージを確認してください。

下記に実行する一例を示します。

```bash
sudo apt-get install libxtst6
sudo apt-get install libegl1-mesa-dev
sudo apt-get install libgles2-mesa
```

### Raspberry-Pi-OS で Raspberry Pi 用カメラなどの CSI カメラを利用する場合

これは USB カメラを利用する場合は不要なオプションです。

raspi-config で Camera を Enable にしてください。

加えて、以下のコマンドを実行してください

```bash
sudo modprobe bcm2835-v4l2 max_video_width=2592 max_video_height=1944
```

## 使ってみる

[USE_TEST.md](USE_TEST.md) をご確認ください。

## ビデオデバイスの指定

ビデオデバイスの指定については [LINUX_VIDEO_DEVICE.md](LINUX_VIDEO_DEVICE.md) をご確認ください。

## Raspberry Pi 向けの追加のオプション

### --force-i420

`--force-i420` は Raspberry Pi 専用カメラ用では MJPEG を使うとパフォーマンスが落ちるため HD 以上の解像度でも MJPEG にせず強制的に I420 でキャプチャーします。
USB カメラでは逆にフレームレートが落ちるため使わないでください。

```bash
./momo --force-i420 --no-audio-device test
```

## Raspberry Pi 専用カメラでが利用できない

Momo 2023.1.0 から Raspberry Pi OS (64 bit) でのみ Raspberry Pi 専用カメラ（CSI 接続のカメラ）が利用できるようになりました。

### --use-libcamera

`--use-libcamera` は Raspberry Pi 専用カメラを利用するためのオプションです。

```bash
./momo --use-libcamera --no-audio-device test
```

## Raspberry Pi 専用カメラでパフォーマンスが出ない

### --hw-mjpeg-decoder

MJPEG のハードウェアデコーダーの利用を検討してみてください。
`--hw-mjpeg-decoder` は ハードウェアによるビデオのリサイズをします。

```bash
./momo --hw-mjpeg-decoder true --no-audio-device test
```

### Raspberry Pi の設定を見直す

[Raspberry-Pi-OS で Raspberry Pi 用カメラなどの CSI カメラを利用する場合](#raspberry-pi-os-で-raspberry-pi-用カメラなどの-csi-カメラを利用する場合) を確認してください。
特に `max_video_width=2592 max_video_height=1944` が記載されていなければ高解像度時にフレームレートが出ません。

### オプションを見直す

Raspberry Pi 用カメラ利用時には `--hw-mjpeg-decoder=true --force-i420` オプションを併用すると CPU 使用率が下がりフレームレートが上がります。例えば、 Raspberry Pi Zero の場合には

```bash
./momo --resolution=HD --force-i420 --hw-mjpeg-decoder=true test
```

がリアルタイムでの最高解像度設定となります。

## USB カメラでパフォーマンスが出ない

### --hw-mjpeg-decoder

一部の MJPEG に対応した USB カメラを使用している場合、 `--hw-mjpeg-decoder` は ハードウェアによるビデオのリサイズ と MJPEG をハードウェアデコードします。

```bash
./momo --hw-mjpeg-decoder true --no-audio-device test
```

### Raspberry Pi で USB カメラ利用時に --hw-mjpeg-decoder を使ってもフレームレートが出ない

USB カメラ利用時には `--hw-mjpeg-decoder` を使わない方がフレームレートは出ます。しかし `--hw-mjpeg-decoder` を使って CPU 使用率を下げた状態で利用したい場合は /boot/config.txt の末尾に下記を追記してください.

bookworm 以降は `/boot/firmware/config.txt` に追記してください。

```text
gpu_mem=256
force_turbo=1
avoid_warnings=2
```

この設定であれば HD は 30fps, FHD では 15fps 程度の性能を発揮します。
