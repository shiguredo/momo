# Ubuntu 24.04 x86_64 で Momo を使ってみる

## Ubuntu 24.04 x86_64 向けのバイナリは以下にて提供しています

<https://github.com/shiguredo/momo/releases> にて最新版のバイナリをダウンロードしてください。

- バイナリは、 `momo-<VERSION>_ubuntu-24.04_x86_64.tar.gz` を利用してください

## ダウンロードしたパッケージ、解凍後の構成

```bash
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

下記を実行してください

```bash
sudo apt-get update
sudo apt-get upgrade
sudo apt-get install libdrm2 libva2 libva-drm2
```

oneVPL を利用したい場合は [VPL.md](VPL.md) を御覧ください。

## 実行権限の付与

ダウンロードした momo の実行ファイルに実行権限を付与してください

```bash
chmod a+x ./momo
```

## 動かしてみる

動かし方については [USE_TEST.md](USE_TEST.md) を御覧ください。

## ビデオデバイスの指定

ビデオデバイスの指定については [LINUX_VIDEO_DEVICE.md](LINUX_VIDEO_DEVICE.md) をご確認ください。
