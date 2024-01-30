# WebRTC Native Client Momo

[![libwebrtc](https://img.shields.io/badge/libwebrtc-m119.6045-blue.svg)](https://chromium.googlesource.com/external/webrtc/+/branch-heads/6045)
[![GitHub tag (latest SemVer)](https://img.shields.io/github/tag/shiguredo/momo.svg)](https://github.com/shiguredo/momo)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![Actions Status](https://github.com/shiguredo/momo/workflows/daily-build-workflow/badge.svg)](https://github.com/shiguredo/momo/actions)

## About Shiguredo's open source software

We will not respond to PRs or issues that have not been discussed on Discord. Also, Discord is only available in Japanese.

Please read https://github.com/shiguredo/oss/blob/master/README.en.md before use.

## 時雨堂のオープンソースソフトウェアについて

利用前に https://github.com/shiguredo/oss をお読みください。

## WebRTC Native Client Momo について

WebRTC Native Client Momo は libwebrtc を利用しブラウザなしで様々な環境で動作する WebRTC ネイティブクライアントです。

https://momo.shiguredo.jp/

### ハードウェアエンコーダーへの対応

- [Raspberry Pi](https://www.raspberrypi.org/) の GPU に積まれている H.264 ハードウェアエンコーダー機能を利用することが可能です
- [NVIDIA Jetson](https://www.nvidia.com/ja-jp/autonomous-machines/embedded-systems/) に搭載されている VP8 や VP9 や H.264 ハードウェアエンコーダー機能を利用することで 4K@30 での配信が可能です
- Apple macOS に搭載されている H.264 ハードウェアアクセラレーター機能を [VideoToolbox](https://developer.apple.com/documentation/videotoolbox) 経由で利用することが可能です
- NVIDIA グラフィックスカードに搭載されているハードウェアアクセラレーター機能を [NVIDIA VIDEO CODEC SDK](https://developer.nvidia.com/nvidia-video-codec-sdk) 経由で利用することが可能です
- [Intel Quick Sync Video](https://www.intel.co.jp/content/www/jp/ja/architecture-and-technology/quick-sync-video/quick-sync-video-general.html) を [Intel Media SDK](https://www.intel.com/content/www/us/en/developer/tools/media-sdk/overview.html) 経由で Windows x86_64 と Ubuntu x86_64 にて VP9 / H.264 ハードウェアアクセラレーター機能を利用することが可能です

### 4K 30fps での配信

Momo はハードウェアエンコーダーを利用することで WebRTC で 4K 60fps の配信を実現可能です

### サイマルキャストへの対応

Momo は Sora モード利用時にサイマルキャスト（複数画質の同時配信）に対応しています。

### データチャネル経由でのシリアルの読み書き

Momo はデータチャネルを利用しシリアルに直接読み書きが可能です。信頼性より低遅延を優先したい場合の利用を想定しています。

### SDL を利用した音声や映像の受信

Momo を GUI 環境で利用した場合、[Simple DirectMedia Layer](https://www.libsdl.org/) を利用して音声や映像の受信を行うことができます。

### AV1 への対応

AV1 の送受信に対応済みです。

### クライアント証明書への対応

Momo は Sora モード利用時にクライアント証明書に対応しています。

## 動画

[WebRTC Native Client Momo と Jetson Nano で 4K@30 配信](https://www.youtube.com/watch?v=z05bWtsgDPY)

## OpenMomo プロジェクトについて

OpenMomo は WebRTC Native Client Momo をオープンソースとして公開し継続的に開発を行っていくプロジェクトです。
ブラウザやスマートフォン以外からの WebRTC をいろいろな用途で使ってもらえればと思っています。

詳細については下記をご確認ください。

[OpenMomo プロジェクト](https://gist.github.com/voluntas/51c67d0d8ce7af9f24655cee4d7dd253)

また Momo についてのつぶやきは以下にまとめてあります。

https://gist.github.com/voluntas/51c67d0d8ce7af9f24655cee4d7dd253#twitter

## 既知の問題について

[既知の問題に対する解決方針](https://github.com/shiguredo/momo/issues/89)

## バイナリ提供について

以下からダウンロードが可能です。

https://github.com/shiguredo/momo/releases

## 動作環境

- Raspberry Pi OS (64bit) ARMv8
    - Raspberry Pi 4
    - Raspberry Pi 3
    - Raspberry Pi 2
    - Raspberry Pi Zero 2 W
- Raspberry Pi OS (32bit) ARMv7
    - Raspberry Pi 4
    - Raspberry Pi 3
    - Raspberry Pi 2
    - Raspberry Pi Zero 2 W
- Raspberry Pi OS (32bit) ARMv6
    - Raspberry Pi Zero
    - Raspberry Pi 1
- Ubuntu 20.04 x86_64
- Ubuntu 22.04 x86_64
- Ubuntu 20.04 ARMv8 Jetson
    - [NVIDIA Jetson AGX Orin](https://www.nvidia.com/ja-jp/autonomous-machines/embedded-systems/jetson-orin/)
    - [NVIDIA Jetson AGX Xavier](https://www.nvidia.com/ja-jp/autonomous-machines/embedded-systems/jetson-agx-xavier/)
    - [NVIDIA Jetson Xavier NX](https://www.nvidia.com/ja-jp/autonomous-machines/embedded-systems/jetson-xavier-nx/)
- macOS 12 arm64 以降
- Windows 10.1809 x86_64 以降

### 非対応

- macOS x86_64
- Ubuntu 20.04 ARMv8 Jetson
    - [NVIDIA Jetson Orin Nano](https://www.nvidia.com/ja-jp/autonomous-machines/embedded-systems/jetson-orin/)
        - Jetson Orin Nano は HWA を詰んでいないので対応はしません
- Ubuntu 18.04 ARMv8 Jetson
    - [NVIDIA Jetson Nano](https://www.nvidia.com/ja-jp/autonomous-machines/embedded-systems/jetson-nano/)
    - [NVIDIA Jetson Xavier NX](https://www.nvidia.com/ja-jp/autonomous-machines/embedded-systems/jetson-xavier-nx/)
    - [NVIDIA Jetson AGX Xavier](https://www.nvidia.com/ja-jp/autonomous-machines/embedded-systems/jetson-agx-xavier/)

## 使ってみる

Momo を使ってみたい人は [USE.md](doc/USE.md) をお読みください。

## ビルドする

- Linux 版 Momo のビルドしたい人は [BUILD_LINUX.md](doc/BUILD_LINUX.md) をお読みください
- macOS 版 Momo のビルドしたい人は [BUILD_MACOS.md](doc/BUILD_MACOS.md) をお読みください
- Windows 版 Momo のビルドしたい人は [BUILD_WINDOWS.md](doc/BUILD_WINDOWS.md) をお読みください

## パッケージを作成する

パッケージ作成したい人は [PACKAGE.md](doc/PACKAGE.md) をお読みください。

## FAQ

[FAQ.md](doc/FAQ.md) をお読みください。

## ライセンス

Apache License 2.0

```
Copyright 2015-2024, tnoho (Original Author)
Copyright 2018-2024, Shiguredo Inc.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
```

## 優先実装

優先実装とは Sora のライセンスを契約頂いているお客様限定で Momo の実装予定機能を有償にて前倒しで実装することです。

- Windows 版 OSS 化
    - [スロースネットワークス株式会社](http://www.sloth-networks.co.jp) 様
- WebRTC's Statistics 対応
    - 現時点では企業名非公開
- Windows 版 Momo NVIDIA VIDEO CODEC SDK 対応
    - [スロースネットワークス株式会社](http://www.sloth-networks.co.jp) 様
- Linux 版 Momo NVIDIA VIDEO CODEC SDK 対応
    - [株式会社オプティム](https://www.optim.co.jp/) 様
- Windows / Linux 版 スクリーンキャプチャ対応
    - [スロースネットワークス株式会社](http://www.sloth-networks.co.jp) 様

### 優先実装が可能な機能一覧

**詳細は Discord やメールなどでお気軽にお問い合わせください**

- リリース
- H.265 (HEVC) HWA 対応
    - macOS Video Toolbox
    - NVIDIA Jetson 
        - Ubuntu 20.04
        - Ubuntu 22.04
    - NVIDIA Video Codec SDK
        - Windows
        - Ubuntu
- OpenH264 対応
    - Windows
    - Ubuntu
- oneVPL 対応
- AV1 対応
    - Windows
- Windows / macOS 署名対応

## Momo についての電子書籍

Momo の原作者である @tnoho が書いた Momo のノウハウが沢山詰まった本が販売されています。

[WebRTCをブラウザ外で使ってブラウザでできることを増やしてみませんか?\(電子版\) \- でんでんらぼ \- BOOTH](https://tnoho.booth.pm/items/1572872)

## サポートについて

### Discord

- **サポートしません**
- アドバイスします
- フィードバック歓迎します

最新の状況などは Discord で共有しています。質問や相談も Discord でのみ受け付けています。

https://discord.gg/shiguredo

### バグ報告

Discord へお願いします。

### 有料でのテクニカルサポートについて

WebRTC Native Client に対する有料でのテクニカルサポート契約については WebRTC SFU Sora ライセンス契約をしているお客様が前提となります。

- Momo のテクニカルサポート
- OSS 公開前提での Momo への機能追加

## H.264 のライセンス費用について

H.264 ハードウェアエンコーダー **のみ** を利用している Momo 単体の配布においてはライセンス費用は不要ですが、
ハードウェアとセットで配布する場合はライセンス費用を支払う必要があります。

ただし、 Raspberry Pi においては H.264 のライセンスがハードウェア費用に含まれているため、
配布時にライセンス費用を支払う必要はありません。

詳細については [MPEG LA](https://www.mpegla.com/) まで問い合わせる事をおすすめします。

- Raspberry Pi のハードウェアエンコーダーのライセンス費用は Raspberry Pi の価格に含まれています
    - https://www.raspberrypi.org/forums/viewtopic.php?t=200855
- Apple のライセンス費用は個人利用および非商用利用目的に限るため、配布においては別途、団体との契約が必要
    - https://store.apple.com/Catalog/Japan/Images/EA0270_QTMPEG2.html
- AMD ビデオカードのハードウェアエンコーダーのライセンス費用は別途、団体との契約が必要
    - https://github.com/GPUOpen-LibrariesAndSDKs/AMF/blob/master/amf/doc/AMF_API_Reference.pdf
- NVIDIA ビデオカードのハードウェアエンコーダーのライセンス費用は別途、団体との契約が必要
    - https://developer.download.nvidia.com/designworks/DesignWorks_SDKs_Samples_Tools_License_distrib_use_rights_2017_06_13.pdf
- NVIDIA Jetson Nano のハードウェアエンコーダーのライセンス費用は別途、団体との契約が必要
    - [NVIDIA Jetson Nano 搭載の H\.264/H\.265 ハードウェアエンコーダーのライセンスについて](https://medium.com/@voluntas/nvidia-jetson-nano-%E6%90%AD%E8%BC%89%E3%81%AE-h-264-h-265-%E3%83%8F%E3%83%BC%E3%83%89%E3%82%A6%E3%82%A7%E3%82%A2%E3%82%A8%E3%83%B3%E3%82%B3%E3%83%BC%E3%83%80%E3%81%AE%E3%83%A9%E3%82%A4%E3%82%BB%E3%83%B3%E3%82%B9%E3%81%AB%E3%81%A4%E3%81%84%E3%81%A6-ca207af302ee)
- Intel Quick Sync Video のハードウェアエンコーダーライセンス費用は別途、団体との契約が必要
    - [QuickSync \- H\.264 patent licensing fees \- Intel Community](https://community.intel.com/t5/Media-Intel-oneAPI-Video/QuickSync-H-264-patent-licensing-fees/td-p/921396)


