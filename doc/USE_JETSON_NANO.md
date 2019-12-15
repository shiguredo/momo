# Jetson Nano で Momo を使ってみる

## Jetson Nano を購入する

- [Jetson Nano 開発者キット \- スイッチサイエンス](https://www.switch-science.com/catalog/5433/)
    - 単体はこちらがおすすめ
- [NVIDIA NV\-JT\-N001\-CSK32 NVIDIA Jetson Nanoコンプリートスターターキット\(32GB\)](https://www.sengoku.co.jp/mod/sgk_cart/detail.php?code=EEHD-5FXJ)
    - 全部入はこちらがおすすめ
- [【OLIOSPECオリジナル】NVIDIA Jetson Nano Developer kit用ケース\(長尾製作所製\) \| すべての商品 \| OLIOSPEC](https://www.oliospec.com/shopdetail/000000008491/)
    - ケースはこちらがおすすめ
- [Noctua NF-A4x10 5V PWMサイレントファン40 mmプレミアムブラウン/ベージュ](https://amazon.co.jp/dp/B07DXS86G7)
    - ファンはこちらがおすすめ

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

## 4K@30 を出すためにやること

### 実行時のコマンドについて

`--fixed-resolution` を外してみてください。4Kの時には `--fixed-resolution` オプションを使うとレートが安定しない傾向があります。

### --use-native オプション利用時のハングアップについて

Jetson Nano のライブラリにバグがあるため、 `/usr/lib/aarch64-linux-gnu/tegra/libnvjpeg.so` を下記の記事で配布されているものに置き換えてください

下記のコマンドの実行結果でパッチが異なります

> cat /etc/nv_tegra_release | head -1

`# R32 (release), REVISION: 1.0` の場合は [こちら](https://devtalk.nvidia.com/default/topic/1050162/jetson-nano/r32-1-0-mmapi-and-decodetofd-leak-memory-/)

`# R32 (release), REVISION: 2.1` の場合は [こちら](https://devtalk.nvidia.com/default/topic/1060896/jetson-tx2/jetpack-4-2-1-nvjpeg-leaking/)

を適用してください

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
