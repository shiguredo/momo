# WebRTC Native Client Momo

[![libwebrtc](https://img.shields.io/badge/libwebrtc-m128.6613-blue.svg)](https://chromium.googlesource.com/external/webrtc/+/branch-heads/6613)
[![GitHub tag (latest SemVer)](https://img.shields.io/github/tag/shiguredo/momo.svg)](https://github.com/shiguredo/momo)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![Actions Status](https://github.com/shiguredo/momo/workflows/daily-build-workflow/badge.svg)](https://github.com/shiguredo/momo/actions)

## About Shiguredo's open source software

We will not respond to PRs or issues that have not been discussed on Discord. Also, Discord is only available in Japanese.

Please read <https://github.com/shiguredo/oss/blob/master/README.en.md> before use.

## 時雨堂のオープンソースソフトウェアについて

利用前に <https://github.com/shiguredo/oss> をお読みください。

## WebRTC Native Client Momo について

WebRTC Native Client Momo は libwebrtc を利用しブラウザなしで様々な環境で動作する WebRTC ネイティブクライアントです。

<https://momo.shiguredo.jp/>

### ハードウェアアクセラレーターへの対応

- Intel グラフィックスチップに搭載されているハードウェアアクセラレーター機能を [Intel VPL](https://www.intel.com/content/www/us/en/developer/tools/vpl/overview.html) 経由で Windows x86_64 と Ubuntu x86_64 にてハードウェアアクセラレーター機能を利用することが可能です
  - VP9 /AV1 の送信時、[既知の問題](https://github.com/shiguredo/momo/issues/357) がありますのでご確認ください
  - ハードウェアエンコーダー: VP9 / AV1 / H.264 / H.265
  - ハードウェアデコーダー: VP9 / AV1 / H.264 / H.265
- Apple macOS に搭載されているハードウェアアクセラレーター機能を [Apple VideoToolbox](https://developer.apple.com/documentation/videotoolbox) 経由で利用することができます
  - ハードウェアエンコーダー: H.264 / H.265
  - ハードウェアデコーダー: H.264 / H.265
- NVIDIA グラフィックスカードに搭載されているハードウェアアクセラレーター機能を [NVIDIA Video Codec SDK](https://developer.nvidia.com/nvidia-video-codec-sdk) 経由で利用することができます
  - ハードウェアエンコーダー: VP9 / AV1 / H.264 / H.265
  - ハードウェアデコーダー: VP9 / AV1 / H.264 / H.265
- [NVIDIA Jetson](https://www.nvidia.com/ja-jp/autonomous-machines/embedded-systems/) に搭載されているハードウェアアクセラレーター機能を [Jetson JetPack SDK](https://developer.nvidia.com/embedded/jetpack) 経由で利用することができます
  - ハードウェアエンコーダー: VP9 / AV1 / H.264 / H.265
  - ハードウェアデコーダー: VP9 / AV1 / H.264 / H.265
- [Raspberry Pi](https://www.raspberrypi.org/) の GPU に積まれているハードウェアアクセラレーター機能を利用することができます
  - ハードウェアエンコーダー: H.264
  - ハードウェアデコーダー: H.264

### 4K の配信

Momo はハードウェアエンコーダーを利用することで WebRTC で 4K の配信を実現可能です

### 4K の視聴

Momo はハードウェアデコーダーを利用することで WebRTC で 4K の配信を実現可能です

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

### OpenH264 の利用

Momo は OpenH264 を利用して H.264 のソフトウェアのエンコード/デコードを行うことができます。

## 動画

[WebRTC Native Client Momo と Jetson Nano で 4K@30 配信](https://www.youtube.com/watch?v=z05bWtsgDPY)

## OpenMomo プロジェクトについて

OpenMomo は WebRTC Native Client Momo をオープンソースとして公開し継続的に開発を行っていくプロジェクトです。
ブラウザやスマートフォン以外からの WebRTC をいろいろな用途で使ってもらえればと思っています。

詳細については下記をご確認ください。

[OpenMomo プロジェクト](https://gist.github.com/voluntas/51c67d0d8ce7af9f24655cee4d7dd253)

また Momo についてのつぶやきは以下にまとめてあります。

<https://gist.github.com/voluntas/51c67d0d8ce7af9f24655cee4d7dd253#twitter>

## 既知の問題について

[既知の問題に対する解決方針](https://github.com/shiguredo/momo/issues/89)

## バイナリ提供について

以下からダウンロードが可能です。

<https://github.com/shiguredo/momo/releases>

## 動作環境

- Windows 11 x86_64
- macOS 15 arm64
- macOS 14 arm64
- Ubuntu 24.04 x86_64
- Ubuntu 22.04 x86_64
- Ubuntu 22.04 ARMv8 (NVIDIA Jetson JetPack 6)
  - [NVIDIA Jetson AGX Orin](https://www.nvidia.com/ja-jp/autonomous-machines/embedded-systems/jetson-orin/)
  - [NVIDIA Jetson Orin NX](https://www.nvidia.com/ja-jp/autonomous-machines/embedded-systems/jetson-orin/)
- Raspberry Pi OS bookworm (64bit)
  - Raspberry Pi 4
  - Raspberry Pi 3
  - Raspberry Pi 2 Model B v1.2
  - Raspberry Pi Zero 2 W

### 対応終了

**優先実装にて対応することができます**

- macOS x86_64
- Ubuntu 20.04 ARMv8 (NVIDIA Jetson JetPack 5)
  - [NVIDIA Jetson AGX Orin](https://www.nvidia.com/ja-jp/autonomous-machines/embedded-systems/jetson-orin/)
  - [NVIDIA Jetson AGX Xavier](https://www.nvidia.com/ja-jp/autonomous-machines/embedded-systems/jetson-agx-xavier/)
  - [NVIDIA Jetson Xavier NX](https://www.nvidia.com/ja-jp/autonomous-machines/embedded-systems/jetson-xavier-nx/)
- Ubuntu 18.04 ARMv8 (NVIDIA Jetson JetPack 4)
  - [NVIDIA Jetson Nano](https://www.nvidia.com/ja-jp/autonomous-machines/embedded-systems/jetson-nano/)
  - [NVIDIA Jetson Xavier NX](https://www.nvidia.com/ja-jp/autonomous-machines/embedded-systems/jetson-xavier-nx/)
  - [NVIDIA Jetson AGX Xavier](https://www.nvidia.com/ja-jp/autonomous-machines/embedded-systems/jetson-agx-xavier/)
- Raspberry Pi OS (32bit)

## 使ってみる

Momo を使ってみたい人は [USE.md](doc/USE.md) をお読みください。

## ビルドする

- Momo をビルドしたい、またはパッケージ作成したい人は [BUILD.md](doc/BUILD.md) をお読みください

## FAQ

[FAQ.md](doc/FAQ.md) をお読みください。

## ライセンス

Apache License 2.0

```text
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

## OpenH264

<https://www.openh264.org/BINARY_LICENSE.txt>

```text
"OpenH264 Video Codec provided by Cisco Systems, Inc."
```

## 優先実装

優先実装とは Sora のライセンスを契約頂いているお客様限定で Momo の実装予定機能を有償にて前倒しで実装することです。

- Windows 版 OSS 化
  - [スロースネットワークス株式会社](http://www.sloth-networks.co.jp) 様
- WebRTC's Statistics 対応
  - 現時点では企業名非公開
- Windows 版 Momo NVIDIA Video Codec SDK 対応
  - [スロースネットワークス株式会社](http://www.sloth-networks.co.jp) 様
- Linux 版 Momo NVIDIA Video Codec SDK 対応
  - [株式会社オプティム](https://www.optim.co.jp/) 様
- Windows / Linux 版 スクリーンキャプチャ対応
  - [スロースネットワークス株式会社](http://www.sloth-networks.co.jp) 様

### 優先実装が可能な機能一覧

**こちらに掲載していない機能でも対応できる場合がありますのでまずはお問い合わせください**

- Windows 11 arm64
- Ubuntu 20.04 arm64 (NVIDIA Jetson JetPack 5)

## Momo についての電子書籍

Momo の原作者である @tnoho が書いた Momo のノウハウが沢山詰まった本が販売されています。

[WebRTC をブラウザ外で使ってブラウザでできることを増やしてみませんか?\(電子版\) \- でんでんらぼ \- BOOTH](https://tnoho.booth.pm/items/1572872)

## サポートについて

### Discord

- **サポートしません**
- アドバイスします
- フィードバック歓迎します

最新の状況などは Discord で共有しています。質問や相談も Discord でのみ受け付けています。

<https://discord.gg/shiguredo>

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

詳細については [Via LA Licensing](https://www.via-la.com/) まで問い合わせる事をおすすめします。

Momo の H.264 対応は [Via LA Licensing](https://www.via-la.com/) (旧 MPEG-LA) に連絡を取り、ロイヤリティの対象にならないことを確認しています。

> 時雨堂がエンドユーザーの PC /デバイスに既に存在する AVC / H.264 エンコーダー/デコーダーに依存する製品を提供する場合は、
> ソフトウェア製品は AVC ライセンスの対象外となり、ロイヤリティの対象にもなりません。

- Raspberry Pi のハードウェアエンコーダーのライセンス費用は Raspberry Pi の価格に含まれています
  - <https://www.raspberrypi.org/forums/viewtopic.php?t=200855>
- Apple のライセンス費用は個人利用および非商用利用目的に限るため、配布においては別途、団体との契約が必要
  - <https://store.apple.com/Catalog/Japan/Images/EA0270_QTMPEG2.html>
- AMD ビデオカードのハードウェアエンコーダーのライセンス費用は別途、団体との契約が必要
  - <https://github.com/GPUOpen-LibrariesAndSDKs/AMF/blob/master/amf/doc/AMF_API_Reference.pdf>
- NVIDIA ビデオカードのハードウェアエンコーダーのライセンス費用は別途、団体との契約が必要
  - <https://developer.download.nvidia.com/designworks/DesignWorks_SDKs_Samples_Tools_License_distrib_use_rights_2017_06_13.pdf>
- NVIDIA Jetson Nano のハードウェアエンコーダーのライセンス費用は別途、団体との契約が必要
  - [NVIDIA Jetson Nano 搭載の H\.264/H\.265 ハードウェアエンコーダーのライセンスについて](https://medium.com/@voluntas/nvidia-jetson-nano-%E6%90%AD%E8%BC%89%E3%81%AE-h-264-h-265-%E3%83%8F%E3%83%BC%E3%83%89%E3%82%A6%E3%82%A7%E3%82%A2%E3%82%A8%E3%83%B3%E3%82%B3%E3%83%BC%E3%83%80%E3%81%AE%E3%83%A9%E3%82%A4%E3%82%BB%E3%83%B3%E3%82%B9%E3%81%AB%E3%81%A4%E3%81%84%E3%81%A6-ca207af302ee)
- Intel Quick Sync Video のハードウェアエンコーダーライセンス費用は別途、団体との契約が必要
  - [QuickSync \- H\.264 patent licensing fees \- Intel Community](https://community.intel.com/t5/Media-Intel-oneAPI-Video/QuickSync-H-264-patent-licensing-fees/td-p/921396)

## H.265 のライセンス費用について

H.265 ハードウェアエンコーダー **のみ** を利用している Momo 単体の配布においてはライセンス費用は不要ですが、
ハードウェアとセットで配布する場合はライセンス費用を支払う必要があります。

Momo の H.265 対応は以下の二つの団体に連絡を取り、H.265 ハードウェアアクセラレーターのみを利用し、
H.265 が利用可能なバイナリを配布する事は、ライセンスが不要であることを確認しています。

また、H.265 のハードウェアアクセラレーターのみを利用した H.265 対応の Momo を OSS で公開し、
ビルド済みバイナリを配布する事は、ライセンスが不要であることも確認しています。

- [Access Advance](https://accessadvance.com/ja/)
- [Via LA Licensing](https://www.via-la.com/)

## NVDIA Video Codec SDK

<https://docs.nvidia.com/video-technologies/video-codec-sdk/12.0/index.html>

```test
“This software contains source code provided by NVIDIA Corporation.”
```
