# Jetson Nano で Momo を使ってみる

## Momo について

Momo は CUI で動かせる WebRTC クライアントです。
Jetson Nano 上で動作させることで H.264 で圧縮された 4K の映像を 1 秒以内でブラウザに配信可能にします。

## Jetson Nano 向けのバイナリは以下にて提供しています

https://github.com/shiguredo/momo/releases にて最新版の Jetson Nano 向けバイナリをダウンロードしてください。

```
momo-<version>_ubuntu-18.04_armv8_jetson_nano.tar.gz
```

## test モードを利用して繋ぐ

Momo 自体がシグナリングサーバの機能を持っている、つ test モードを利用して動かしてみてください。

```shell
$ ./momo --no-audio --port 8080 test
```

momo 起動後 http://[Jetson Nano の IP アドレス]:8080/html/test.html にアクセスして接続してみてください。

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
