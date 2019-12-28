# WebRTC Native Client Momo

[![GitHub tag (latest SemVer)](https://img.shields.io/github/tag/shiguredo/momo.svg)](https://github.com/shiguredo/momo)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![Actions Status](https://github.com/shiguredo/momo/workflows/daily-build-workflow/badge.svg)](https://github.com/shiguredo/momo/actions)

## About Support

We check PRs or Issues only when written in JAPANESE.
In other languages, we won't be able to deal with them. Thank you for your understanding.

## WebRTC Native Client Momo について

WebRTC Native Client Momo は libwebrtc を利用しブラウザなしで様々な環境で動作する WebRTC ネイティブクライアントです。

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

## ayame モードを利用して繋ぐ

Ayame Lite は無料で利用可能なシグナリングサーバです。

ルーム ID は open-momo を利用していますが、実際に利用する場合は推測されにくい ID を利用してください。

```shell
$ ./momo --no-audio ayame wss://ayame-lite.shiguredo.jp/signaling open-momo
```

Ayame Web SDK のオンラインサンプルを利用して確認します。 URL の引数にルーム ID を指定してアクセスします。

```
https://openayame.github.io/ayame-web-sdk-samples/recvonly.html?roomId=open-momo
```
