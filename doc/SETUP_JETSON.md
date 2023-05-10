# Jetson シリーズで Momo を使ってみる

もし Jetson シリーズを購入する場合は [BUY_JETSON.md](BUY_JETSON.md) を参考にしてください。

## Jetson シリーズでは JetPack 5.1.1 以上の利用を前提としています

[JetPack 5.1.1 is now live \- Jetson & Embedded Systems / Announcements \- NVIDIA Developer Forums](https://forums.developer.nvidia.com/t/jetpack-5-1-1-is-now-live/247862/1)

## Jetson シリーズ向けのバイナリは以下にて提供しています

https://github.com/shiguredo/momo/releases にて最新版のバイナリをダウンロードしてください。

- `momo-<version>_ubuntu-20.04_armv8_jetson_xavier.tar.gz`
    - Jetson AGX Orin , Jetson AGX Xavier または Jetson Xavier NX

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

## Jetson 向けの追加のオプション

### --hw-mjpeg-decoder

`--hw-mjpeg-decoder` は ハードウェアによるビデオのリサイズ と USB カメラ用の場合 MJPEG をハードウェアデコードします。
Jetson シリーズではデフォルトで `--hw-mjpeg-decoder=true` です。 ハードウェアデコードに対応していないコーデックを利用したい場合は `--hw-mjpeg-decoder=false` を指定してください。

```shell
$ ./momo --hw-mjpeg-decoder=true --no-audio-device test
```

## 4K@30fps を出すためにやること

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

## 4K@30fps の実行例

ここでは Jetson AGX Orin を使って 4K@30fps を実行する方法を記載します。

### 事前確認

4K@30fps のコマンドを実行する前に準備が完了しているか事前に確認をします。

- Jetson AGX Orin で momo を使うためのセットアップが全て完了している
- 4K@30fps が可能なカメラがセットされている
- Sora/Sora Labo のアカウントの用意がある

### 実行してみる

ここでは利用申請することで法人などで無料で検証可能な [Sora Labo](https://sora-labo.shiguredo.jp/) を利用しています。

Sora Labo の利用申請や使用方法については [Sora Labo のドキュメント](https://github.com/shiguredo/sora-labo-doc)をご確認ください。

コマンド例を以下に記載します。

```shell
$ ./momo --hw-mjpeg-decoder true --framerate 30 --resolution 4K --log-level 2 sora \
    --signaling-url \
        wss://canary.sora-labo.shiguredo.app/signaling \
    --channel-id shiguredo_0_sora \
    --video true --audio true \
    --video-codec-type VP8 --video-bit-rate 15000 \
    --auto --role sendonly \
    --metadata '{"access_token": "xyz"}'
```

コマンド例の構成は以下のようになっています。

- ./momo ~ sora までは momo に対して行うパラメータになっています。
    - `--hw-mjpeg-decoder true` は Hardware Acceleration を有効に設定しています
    - `--framerate 30` は フレームレートを 30 に設定しています
    - `--resolution 4K` は解像度を 4K に設定しています
    - `--log-level 2` は error と warning のログを出力するように設定しています
    - `sora` は Sora モードを利用するように設定しています
- `sora` 以降 2 行目からは Sora との接続のためのパラメータになっています
    - `wss://canary.sora-labo.shiguredo.app/signaling` はシグナリング URL の設定をしています
    - `shiguredo_0_sora` はチャネル ID を設定しています
    - `--video true` は Sora への映像送信を有効に設定しています
    - `--audio true` は Sora への音声送信を有効に設定しています
    - `--video-codec-type VP8` はコーデックを VP8 に設定しています
    - `--video-bit-rate 15000` はビデオビットレートを 1.5Mbps で設定しています
    - `--auto` は Sora との自動接続を有効に設定しています
    - `--role sendonly` は送信時の役割を送信のみで設定しています
    - `--metadata '{"access_token": "xyz"}'` は Sora Labo のアクセストークンをメタデータに設定しています

### 実行結果

実行結果の確認はChrome の `chrome://webrtc-internals` を利用します。

`chrome://webrtc-internals` を確認すると以下のように 4K(3840x2160) で 30 fps が出ていることが確認できます。

[![Image from Gyazo](https://i.gyazo.com/df47a19994982ed963e84d88adf4f407.png)](https://gyazo.com/df47a19994982ed963e84d88adf4f407)

Sora Labo を利用している場合はリモート統計機能を利用することで確認することができます。

[![Image from Gyazo](https://i.gyazo.com/314e5ef5cc6ad4f9ad8583fada720809.png)](https://gyazo.com/314e5ef5cc6ad4f9ad8583fada720809)

### それでも 30fps がでない場合

もう一度 `4K@30 を出すためにやること` を確認してみてください。
