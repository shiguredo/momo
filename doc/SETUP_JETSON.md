# Jetson シリーズで Momo を使ってみる

もし Jetson シリーズを購入する場合は [BUY_JETSON.md](BUY_JETSON.md) を参考にしてください。

## Jetson シリーズでは JetPack 4.4 以上の利用を前提としています

[JetPack 4\.4 \- L4T R32\.4\.3 production release \- Jetson & Embedded Systems / Announcements \- NVIDIA Developer Forums](https://forums.developer.nvidia.com/t/jetpack-4-4-l4t-r32-4-3-production-release/140870)

## Jetson シリーズ向けのバイナリは以下にて提供しています

https://github.com/shiguredo/momo/releases にて最新版のバイナリをダウンロードしてください。

- `momo-<version>_ubuntu-18.04_armv8_jetson_nano.tar.gz`
    - Jetson Nano
- `momo-<version>_ubuntu-18.04_armv8_jetson_xavier.tar.gz`
    - Jetson Xavier NX または Jetson AGX Xavier

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

## 動かしてみる

動かし方について、まずは [USE_TEST.md](USE_TEST.md) をご確認ください。

## ビデオデバイスの指定

ビデオデバイスの指定については [LINUX_VIDEO_DEVICE.md](LINUX_VIDEO_DEVICE.md) をご確認ください。

## Jetson Nano 向けの追加のオプション

### --hw-mjpeg-decoder

`--hw-mjpeg-decoder` は ハードウェアによるビデオのリサイズ と USB カメラ用の場合 MJPEG をハードウェアデコードします。

```shell
$ ./momo --hw-mjpeg-decoder=true --no-audio-device test
```

## 4K@30 を出すためにやること

### 実行時のコマンドについて

`--fixed-resolution` を外してみてください。4Kの時には `--fixed-resolution` オプションを使うとレートが安定しない傾向があります。

### フレームレートが出ない場合

一番多いのは暗い場所で利用しているパターンです。カメラが自動的に露光時間を伸ばすためフレームレートが下がります。部屋を明るくする。もしくはカメラの設定変更が可能な場合はフレームレート優先設定に変更してください。

### [IMX317を搭載した推奨カメラ](https://ja.aliexpress.com/item/32999909513.html) をご利用の場合

> v4l2-ctl --set-ctrl=exposure_auto=1

を実行してカメラの設定を変更してください。 4K 30fps が出力可能な設定は下記のとおりです

```
$ v4l2-ctl --list-ctrls
                     brightness 0x00980900 (int)    : min=-64 max=64 step=1 default=0 value=0
                       contrast 0x00980901 (int)    : min=0 max=95 step=1 default=1 value=1
                     saturation 0x00980902 (int)    : min=0 max=100 step=1 default=60 value=60
                            hue 0x00980903 (int)    : min=-2000 max=2000 step=1 default=0 value=0
 white_balance_temperature_auto 0x0098090c (bool)   : default=1 value=1
                          gamma 0x00980910 (int)    : min=64 max=300 step=1 default=100 value=100
                           gain 0x00980913 (int)    : min=0 max=255 step=1 default=100 value=100
           power_line_frequency 0x00980918 (menu)   : min=0 max=2 default=1 value=1
      white_balance_temperature 0x0098091a (int)    : min=2800 max=6500 step=1 default=4600 value=4600 flags=inactive
                      sharpness 0x0098091b (int)    : min=0 max=7 step=1 default=0 value=0
         backlight_compensation 0x0098091c (int)    : min=0 max=100 step=1 default=64 value=64
                  exposure_auto 0x009a0901 (menu)   : min=0 max=3 default=3 value=1
              exposure_absolute 0x009a0902 (int)    : min=1 max=10000 step=1 default=156 value=156
error 5 getting ext_ctrl Pan (Absolute)
error 5 getting ext_ctrl Tilt (Absolute)
                 focus_absolute 0x009a090a (int)    : min=0 max=1023 step=1 default=0 value=0 flags=inactive
                     focus_auto 0x009a090c (bool)   : default=1 value=1
error 5 getting ext_ctrl Zoom, Absolute
```

## 4K@30 の実行例

ここでは実際に 4K@30 を実行する方法を記載します。

### 事前確認

- Jetson シリーズ向けのセットアップが全て完了していること
- 4K@30fps が可能なカメラであること
- Sora/Sora Labo のアカウントの用意があること

### 実行コマンド

ここでは無料で Sora を試すことのできる [Sora Labo](https://sora-labo.shiguredo.jp/) を利用しています。Sora Labo や Sora での実行についての詳細は [USE_SORA.md](USE_SORA.md) をお読みください。

コマンドは VP8 HWA 有で実行します。

```shell
$ ./momo --hw-mjpeg-decoder true --framerate 30 --resolution FHD --log-level 2 sora wss://sora-labo.shiguredo.jp/signaling shiguredo@sora-labo --video true --audio true --video-codec-type VP8 --video-bit-rate 10000 --auto --role sendonly --multistream true --metadata '{"signaling_key": "xyz"}'
```

### 実行結果

このように 4K(3840x2160) で 30 fps が出ていることが確認できます。

[![Image from Gyazo](https://i.gyazo.com/177519876a497aa3cdf166c3ae5f80d5.png)](https://gyazo.com/177519876a497aa3cdf166c3ae5f80d5)

### それでも 30fps がでない場合

もう一度 `4K@30 を出すためにやること` を確認してみてください。
