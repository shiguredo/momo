# Linux で Momo を使ってみる

## Linux 向けのバイナリは以下にて提供しています

https://github.com/shiguredo/momo/releases にて最新版のバイナリをダウンロードしてください。

- Ubuntu 18.04 を利用する場合は、 `momo-<VERSION>_ubuntu-18.04_armv8_jetson_xavier.tar.gz` を利用してください
- Ubuntu 18.04 を利用する場合は、 `momo-<VERSION>_ubuntu-20.04_armv8_jetson_xavier.tar.gz` を利用してください

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

## 準備

### パッケージのインストール


Ubuntu 20.04 で OneVPL を利用したい場合は以下を実行してください

**※ OneVPL は Ubuntu 20.04 のみ利用可能です**

```
$ sudo apt-get update
$ sudo apt-get upgrade
$ sudo apt install libdrm-dev
$ sudo apt install libva-dev
$ sudo apt install pkg-config
```


## 動かしてみる

動かし方については [USE_TEST.md](USE_TEST.md) を御覧ください。

## ビデオデバイスの指定

ビデオデバイスの指定については [LINUX_VIDEO_DEVICE.md](LINUX_VIDEO_DEVICE.md) をご確認ください。

