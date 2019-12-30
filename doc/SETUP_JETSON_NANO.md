# Jetson Nano で Momo を使ってみる

もし Jetson Nano を購入する場合は [BUY_JETSON_NANO.md](BUY_JETSON_NANO.md) を参考にしてください。

## Jetson Nano では JetPack 4.3 以上の利用を前提としています

`JetPack 4.3 - L4T R32.3.1 released - NVIDIA Developer Forums <https://devtalk.nvidia.com/default/topic/1068583/jetson-nano/jetpack-4-3-l4t-r32-3-1-released/>`_

## Jetson Nano 向けのバイナリは以下にて提供しています

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

## 動かしてみる

動かし方について、まずは [USE_TEST.md](USE_TEST.md) をご確認ください。

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
