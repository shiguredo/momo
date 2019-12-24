# Jetson Nano で Momo を使ってみる

## Momo について

Momo は CUI で動かせる WebRTC クライアントです。
Jetson Nano 上で動作させることで H.264 で圧縮された 4K の映像を 1 秒以内でブラウザに配信可能にします。

## Jetson Nano 向けのバイナリ

https://github.com/shiguredo/momo/releases にて最新版の Jetson Nano 向けバイナリをダウンロードしてください。

```
momo-<version>_ubuntu-18.04_armv8_jetson_nano.tar.gz
```

## test モードを利用してつないで見る

Momo 自体がシグナリングサーバの機能を持っている、つ test モードを利用して動かしてみてください。

```shell
$ ./momo --no-audio --port 8080 test
```

momo 起動後 http://[Jetson Nano の IP アドレス]:8080/html/test.html にアクセスして接続してみてください。
