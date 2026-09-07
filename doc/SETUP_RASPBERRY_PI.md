# Raspberry Pi (Raspberry Pi OS) で Momo を使ってみる

## 注意

Raspberry Pi OS のレガシー版には対応しておりません。最新版の Raspberry Pi OS (64bit) を利用してください

## Raspberry Pi 向けのバイナリは以下にて提供しています

<https://github.com/shiguredo/momo/releases> にて最新版のバイナリをダウンロードしてください。

- Raspberry Pi OS 64 bit を利用する場合は、 `momo-<VERSION>_raspberry-pi-os_armv8.tar.gz` を利用してください

## ダウンロードしたパッケージ、解凍後の構成

解凍後の構成は [BUILD.md のパッケージ解凍後の構成](BUILD.md#パッケージ解凍後の構成) を参照してください。

## 準備

### パッケージのインストール

下記を実行してください。

```bash
sudo apt-get update
sudo apt-get upgrade
sudo apt-get install libnspr4 libnss3
sudo apt-get install libcamera0.6
```

#### Raspberry Pi OS Lite を利用する場合

Raspberry Pi Lite では映像に関するパッケージが入っていないため、`ldd ./momo | grep not` を実行し、不足しているパッケージを確認してください。

下記に実行する一例を示します。

```bash
sudo apt-get install libxtst6
sudo apt-get install libegl1-mesa-dev
sudo apt-get install libgles2-mesa
```

### Raspberry Pi OS で Raspberry Pi 用カメラなどの CSI カメラを利用する場合

USB カメラを利用する場合、この手順は不要です。

Raspberry Pi OS bookworm 以降では従来のカメラシステムは利用できません。CSI カメラは `--use-libcamera` を指定してください。詳細は [LIBCAMERA.md](LIBCAMERA.md) を参照してください。

## 使ってみる

[USE_P2P.md](USE_P2P.md) をご確認ください。

## ビデオデバイスの指定

ビデオデバイスの指定については [LINUX_VIDEO_DEVICE.md](LINUX_VIDEO_DEVICE.md) をご確認ください。

## Raspberry Pi 向けの追加のオプション

### --force-i420

`--force-i420` は Raspberry Pi 専用カメラ用では MJPEG を使うとパフォーマンスが落ちるため HD 以上の解像度でも MJPEG にせず強制的に I420 でキャプチャーします。
USB カメラでは逆にフレームレートが落ちるため使わないでください。

```bash
./momo --force-i420 --no-audio-device p2p
```

## Raspberry Pi 専用カメラが利用できない

Momo 2023.1.0 から Raspberry Pi OS (64 bit) でのみ Raspberry Pi 専用カメラ（CSI 接続のカメラ）が利用できるようになりました。

### --use-libcamera

`--use-libcamera` は Raspberry Pi 専用カメラを利用するためのオプションです。

```bash
./momo --use-libcamera --no-audio-device p2p
```

## Raspberry Pi 専用カメラでパフォーマンスが出ない

### --hw-mjpeg-decoder

MJPEG のハードウェアデコーダーの利用を検討してみてください。
`--hw-mjpeg-decoder` は ハードウェアによるビデオのリサイズをします。

```bash
./momo --hw-mjpeg-decoder true --no-audio-device p2p
```

### オプションを見直す

Raspberry Pi 用カメラ利用時には `--hw-mjpeg-decoder=true --force-i420` オプションを併用すると CPU 使用率が下がりフレームレートが上がります。例えば、 Raspberry Pi Zero の場合には

```bash
./momo --resolution=HD --force-i420 --hw-mjpeg-decoder=true p2p
```

がリアルタイムでの最高解像度設定となります。

## USB カメラでパフォーマンスが出ない

### --hw-mjpeg-decoder

一部の MJPEG に対応した USB カメラを使用している場合、 `--hw-mjpeg-decoder` は ハードウェアによるビデオのリサイズ と MJPEG をハードウェアデコードします。

```bash
./momo --hw-mjpeg-decoder true --no-audio-device p2p
```

### Raspberry Pi で USB カメラ利用時に --hw-mjpeg-decoder を使ってもフレームレートが出ない

USB カメラ利用時にフレームレートを出したい場合は `--hw-mjpeg-decoder` を使わないことをおすすめします。ただし CPU 使用率はあがってしまいます。

CPU 使用率を抑えつつフレームレートを出したい場合は、`/boot/firmware/config.txt` の末尾に下記を追記して `--hw-mjpeg-decoder` を指定することで改善することがあります。

bookworm より前のバージョンをご利用の場合は `/boot/config.txt` に追記してください。

```text
gpu_mem=256
force_turbo=1
avoid_warnings=2
```

この設定であれば HD は 30fps, FHD では 15fps 程度の性能を発揮します。
