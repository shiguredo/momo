# Jetson Nano で Momo を使ってみる

## Jetson Nano を購入する

- [Jetson Nano 開発者キット](https://www.switch-science.com/catalog/5433/)
    - [NVIDIA Jetson Nano Developer Kit](https://developer.nvidia.com/embedded/jetson-nano-developer-kit)
    - Jetson Nano 本体はこちらがおすすめ
    - スイッチサイエンス
- [【OLIOSPECオリジナル】NVIDIA Jetson Nano Developer kit用ケース\(長尾製作所製\)](https://www.oliospec.com/shopdetail/000000008491/)
    - ケースはこちらがおすすめ
    - OLIOSPEC
- [Micron MTSD032AHC6MS\-1WT 32GB 産業用microSDカード](https://www.oliospec.com/shop/shopdetail.html?brandcode=000000007374)
    - SD カードはこちらがおすすめ
    - OLIOSPEC
- [Noctua NF-A4x10 5V PWMサイレントファン40 mmプレミアムブラウン/ベージュ](https://amazon.co.jp/dp/B07DXS86G7)
    - [NF A4x10 5V PWM](https://noctua.at/en/nf-a4x10-5v-pwm)
    - ファンはこちらがおすすめ
    - Amazon
- [２４Ｗ級スイッチングＡＣアダプター５Ｖ４Ａ　ＧＦ２４－ＵＳ０５４０](http://akizukidenshi.com/catalog/g/gM-09594/)
    - 電源はこちらがおすすめ
    - 秋月電子通商
- [つまみ付ジャンパーピン（緑）（２．５４ｍｍピッチ）（２０個入）](http://akizukidenshi.com/catalog/g/gP-03894/)
    - ジャンパーピンを持っていない人はこちら
    - 秋月電子通商
 - [TP-Link WiFi 無線LAN 子機 AC600 433Mbps + 200Mbps](https://amazon.co.jp/dp/B07MXHJ6KB)
    - [Archer T2U Nano \| AC600 ナノ 無線LAN子機 \| TP\-Link Japan](https://www.tp-link.com/jp/home-networking/adapter/archer-t2u-nano/)
    - ドライバーのビルドが必要になるが問題なく使える
    - [Jetson Nano で TP\-LINK Archer T2U Nano を使う \- Qiita](https://qiita.com/daisuzu_/items/8d6913f3bda1b7434526)
        - こちらにかかれている `os_dep/linux/usb_intf.c の 300行番台あたりに以下を追加する` は最新版では不要
    - Amazon

## 4K@30 出るカメラの購入する

実際に検証して 4K@30 の出力動作確認が取れているカメラです。

- [高解像度 4 18K カメラモジュール 3840 × 2160 ソニー IMX317 Mjpeg 30fps ミニ Usb ウェブカメラ Web カメラ](https://ja.aliexpress.com/item/32999909513.html)

以下は上のタイプのケースありやレンズが色々選べるタイプです。

- https://ja.aliexpress.com/item/33013268769.html
- https://ja.aliexpress.com/item/33016603918.html
- https://ja.aliexpress.com/item/33012473257.html

色々 4K@30 が出せるカメラを試してきましたが、このカメラが一番安定しています。

## Jetson Nano 向けのバイナリは以下にて提供しています

https://github.com/shiguredo/momo/releases にて最新版のバイナリをダウンロードしてください。

## JetPack 4.3 以上を必ず利用してください

`JetPack 4.3 - L4T R32.3.1 released - NVIDIA Developer Forums <https://devtalk.nvidia.com/default/topic/1068583/jetson-nano/jetpack-4-3-l4t-r32-3-1-released/>`_

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
