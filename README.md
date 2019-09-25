# WebRTC Native Client Momo

[![GitHub tag (latest SemVer)](https://img.shields.io/github/tag/shiguredo/momo.svg)](https://github.com/shiguredo/momo)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![Actions Status](https://github.com/shiguredo/momo/workflows/build-workflow/badge.svg)](https://github.com/shiguredo/momo/actions)

## WebRTC Native Client Momo について

WebRTC Native Client Momo は libwebrtc を利用しブラウザなしで様々な環境で動作する WebRTC ネイティブクライアントです。

### ハードウェアエンコーダへの対応

Raspberry Pi の GPU に積まれている H.264 ハードウェアエンコーダー機能を利用することが可能です。

macOS に積まれている [VideoToolbox](https://developer.apple.com/documentation/videotoolbox) に対応しており、こちらも H.264 ハードウェアエンコーダー機能を利用することが可能です。

Jetson Nano に搭載されている H.264 ハードウェアエンコーダー機能を利用することで 4K@30 での配信が可能です。

[ROS](http://www.ros.org/) ノードとしても利用可能です。

## OpenMomo プロジェクトについて

OpenMomo は WebRTC Native Client Momo をオープンソースとして公開し継続的に開発を行っていくプロジェクトです。
ブラウザやスマートフォン以外からの WebRTC をいろいろな用途で使ってもらえればと思っています。

詳細については下記をご確認ください。

[OpenMomo プロジェクト](https://gist.github.com/voluntas/51c67d0d8ce7af9f24655cee4d7dd253)

## 開発について

Momo はオープンソースソフトウェアですが、開発についてはオープンではありません。
そのためコメントやプルリクエストを頂いてもすぐには採用はしません。

まずは Discord にてご連絡ください。

## 既知の問題について

[既知の問題に対する解決方針](https://github.com/shiguredo/momo/issues/89)

## バイナリ提供について

4 種類のバイナリを配布しています。

- Raspberry Buster ARMv7
- Raspberry Buster ARMv6
- Jetson Nano ARMv8
- macOS 10.14 x86_64

## 動作環境

- Raspbian Buster ARMv7
    - Raspberry Pi 3 B/B+ で動作
- Raspbian Buster ARMv6
    - Raspberry Pi Zero W/WH で動作
- Ubuntu 18.04 x86_64
- Ubuntu 18.04 ARMv8
    - [NVIDIA Jetson Nano](https://www.nvidia.com/ja-jp/autonomous-machines/embedded-systems/jetson-nano/)
    - [ROCK64 –PINE64](https://www.pine64.org/?page_id=7147)
- Ubuntu 16.04 x86_64 ROS Kinetic
    - http://wiki.ros.org/kinetic
- Ubuntu 16.04 ARMv7 ROS Kinetic
    - Raspberry Pi 3 B+
- macOS 10.14 x86_64

## 使ってみる

Momo を使ってみたい人は [USE.md](doc/USE.md) をお読みください。

## ビルドする

Linux 版 Momo のビルドに挑戦したい人は [BUILD_LINUX.md](doc/BUILD_LINUX.md) をお読みください。

## パッケージを作成する

パッケージ作成に挑戦したい人は [PACKAGE.md](doc/PACKAGE.md) をお読みください。

## ライセンス

Apache License 2.0

```
Copyright 2018-2019, Shiguredo Inc, tnoho and melpon and kdxu

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

## サポートについて

WebRTC Native Client Momo に関するバグ報告は GitHub Issues へお願いします。それ以外については Discord へお願いします。

### バグ報告

https://github.com/shiguredo/momo/issues

### Discord

ベストエフォートで運用しています。

https://discord.gg/gmEuZye

### 有料サポートについて

WebRTC Native Client に対する有料でのサポート契約については WebRTC SFU Sora ライセンス契約をしているお客様が前提となります。

## H.264 のライセンス費用について

H.264 ハードウェアエンコーダ **のみ** を利用している Momo 単体の配布においてはライセンス費用は不要ですが、ハードウェアとセットで配布する場合はライセンス費用を支払う必要があります。

ただし、 Raspberry Pi においては H.264 のライセンスがハードウェア費用に含まれているため、配布時にライセンス費用を支払う必要はありません。

詳細については [MPEG LA](https://www.mpegla.com/) まで問い合わせる事をおすすめします。

- Raspberry Pi のハードウェアエンコーダのライセンス費用は Raspberry Pi の価格に含まれています
    - https://www.raspberrypi.org/forums/viewtopic.php?t=200855
- Apple のライセンス費用は個人利用および非商用利用目的に限るため、配布においては別途、団体との契約が必要
    - https://store.apple.com/Catalog/Japan/Images/EA0270_QTMPEG2.html
- AMD ビデオカードのハードウェアエンコーダのライセンス費用は別途、団体との契約が必要
    - https://github.com/GPUOpen-LibrariesAndSDKs/AMF/blob/master/amf/doc/AMF_API_Reference.pdf
- NVIDIA ビデオカードのハードウェアエンコーダのライセンス費用は別途、団体との契約が必要
    - https://developer.download.nvidia.com/designworks/DesignWorks_SDKs_Samples_Tools_License_distrib_use_rights_2017_06_13.pdf
- NVIDIA Jetson Nano のハードウェアエンコーダのライセンス費用は別途、団体との契約が必要
    - [NVIDIA Jetson Nano 搭載の H\.264/H\.265 ハードウェアエンコーダのライセンスについて](https://medium.com/@voluntas/nvidia-jetson-nano-%E6%90%AD%E8%BC%89%E3%81%AE-h-264-h-265-%E3%83%8F%E3%83%BC%E3%83%89%E3%82%A6%E3%82%A7%E3%82%A2%E3%82%A8%E3%83%B3%E3%82%B3%E3%83%BC%E3%83%80%E3%81%AE%E3%83%A9%E3%82%A4%E3%82%BB%E3%83%B3%E3%82%B9%E3%81%AB%E3%81%A4%E3%81%84%E3%81%A6-ca207af302ee)
- Intel Quick Sync Video のハードウェアエンコーダライセンス費用は別途、団体との契約が必要
    - [QuickSync \- H\.264 patent licensing fees](https://software.intel.com/en-us/forums/intel-media-sdk/topic/494720)

## 利用例など

- [libwebRTCでFakeVideoしてみた \- Qiita](https://qiita.com/aikw/items/efb3726eb808a913d9da)
- [ティアフォーにおける自動運転車両の遠隔監視・操縦システムのご紹介 \- Tier IV Tech Blog](https://tech.tier4.jp/entry/2019/01/22/170032)
- [Run WebRTC Native Client Momo on Raspberry Pi 3B \| SHIROKU\.NET](https://shiroku.net/robotics/run-webrtc-native-client-momo-on-raspberry-pi-3b/)
- [WebRTC Native Client Momo がスゲエ – papalagi\.org](https://papalagi.org/blog/archives/635)
- [ラズパイのmomoで音声付きでWebRTCで配信する \- Qiita](https://qiita.com/tetsu_koba/items/33e335fb46f26bbd3431)
- [ラズパイ\+momoでWebRTCで送信するときにマイクの代わりに音声ファイルを使用する \- Qiita](https://qiita.com/tetsu_koba/items/b887c1a0be9f26b795f2)
- [ラズパイのmomoでステレオ音声でWebRTCで配信する \- Qiita](https://qiita.com/tetsu_koba/items/6c07129caa5a08d5d172)

## Momo についての電子書籍

Momo のコア開発者である @tnoho が書いた Momo のノウハウが沢山詰まった本が販売されています。

[WebRTCをブラウザ外で使ってブラウザでできることを増やしてみませんか?\(電子版\) \- でんでんらぼ \- BOOTH](https://tnoho.booth.pm/items/1572872)

## Windows 版について

Windows 版 Momo をビルドするツールを販売しております。興味のある方は以下をご確認ください。

[Windows 版 WebRTC Native Client Momo](https://gist.github.com/voluntas/7af1596557121796123ac7dee9e3f5a4)

## カスタマイズなど

OSS にて公開可能でさらに弊社が Momo に組み込んでも良いと思える機能開発であれば、お仕事としてお受けすることが可能です。

非公開のカスタマイズは時雨堂がお仕事としてお受けする事はできません。ただし Momo の開発を仕事として受けることが可能なコミッターを紹介することが可能です。

まずは momo at shiguredo.jp までお問い合わせください。

- @tnoho
    - Momo のコアコミッターです
- @melpon
    - Momo のビルド全般のリードコミッターです
- @kdxu
    - Momo の Ayame 機能のリードコミッターです
    - [WebRTC Signaling Server Ayame](https://github.com/OpenAyame/ayame) のコアコミッターです
