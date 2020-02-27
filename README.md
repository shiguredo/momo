# WebRTC Native Client Momo

[![GitHub tag (latest SemVer)](https://img.shields.io/github/tag/shiguredo/momo.svg)](https://github.com/shiguredo/momo)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![Actions Status](https://github.com/shiguredo/momo/workflows/daily-build-workflow/badge.svg)](https://github.com/shiguredo/momo/actions)

## About Support

We check PRs or Issues only when written in JAPANESE.
In other languages, we won't be able to deal with them. Thank you for your understanding.

## WebRTC Native Client Momo について

WebRTC Native Client Momo は libwebrtc を利用しブラウザなしで様々な環境で動作する WebRTC ネイティブクライアントです。

https://momo.shiguredo.jp/

### ハードウェアエンコーダへの対応

- Raspberry Pi の GPU に積まれている H.264 ハードウェアエンコーダー機能を利用することが可能です
- macOS に積まれている [VideoToolbox](https://developer.apple.com/documentation/videotoolbox) の H.264 ハードウェアエンコーダー機能を利用することが可能です
- Jetson Nano に搭載されている H.264 ハードウェアエンコーダー機能を利用することで H.264 を 4K@30 での配信が可能です。

### データチャネル経由でのシリアルの読み書き

Momo はデータチャネルを利用しシリアルに直接読み書きが可能です。信頼性より低遅延を優先したい場合の利用を想定しています。

### SDL を利用した音声や映像の受信

Momo を GUI 環境で利用した場合、[Simple DirectMedia Layer](https://www.libsdl.org/) を利用して音声や映像の受信を行うことができます。

### ROS への対応

[ROS](http://www.ros.org/) ノードとしても利用可能です。

## 動画

[WebRTC Native Client Momo と Jetson Nano で 4K@30 配信](https://www.youtube.com/watch?v=z05bWtsgDPY)

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

以下からダウンロードが可能です。

https://github.com/shiguredo/momo/releases

## 動作環境

- Raspbian Buster ARMv7
    - Raspberry Pi 4
    - Raspberry Pi 3
    - Raspberry Pi 2
- Raspbian Buster ARMv6
    - Raspberry Pi Zero
    - Raspberry Pi 1
- Ubuntu 18.04 x86_64
- Ubuntu 18.04 ARMv8 Jetson Nano
    - [NVIDIA Jetson Nano](https://www.nvidia.com/ja-jp/autonomous-machines/embedded-systems/jetson-nano/)
- macOS 10.15 x86_64
- Windows 10.1809 x86_64 以降

### 以下はビルドが通ること以外は確認していません

- Ubuntu 16.04 x86_64 ROS Kinetic
    - http://wiki.ros.org/kinetic
- Ubuntu 16.04 ARMv7 ROS Kinetic
    - 今後削除予定
    - Raspberry Pi 3 B+

## 使ってみる

Momo を使ってみたい人は [USE.md](doc/USE.md) をお読みください。

## ビルドする

- Linux 版 Momo のビルドしたい人は [BUILD_LINUX.md](doc/BUILD_LINUX.md) をお読みください
- macOS 版 Momo のビルドしたい人は [BUILD_MACOS.md](doc/BUILD_MACOS.md) をお読みください
- Windows 版 Momo のビルドしたい人は [BUILD_WINDOWS.md](doc/BUILD_WINDOWS.md) をお読みください

## パッケージを作成する

パッケージ作成したい人は [PACKAGE.md](doc/PACKAGE.md) をお読みください。

## バージョン番号について

```
YYYY.<その年にリリースした回数>
```

## ライセンス

Apache License 2.0

```
Copyright 2015-2020, tnoho (Original Author)
Copyright 2018-2020, Shiguredo Inc.

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

## 有償での優先実装

- Windows 版 OSS 化
    - [スロースネットワークス株式会社](http://www.sloth-networks.co.jp) 様
- WebRTC's Statistics 対応
    - 企業名非公開

## 有償での優先実装が可能な機能一覧

**詳細は momo at shiguredo dot jp までお問い合わせください**

- Jetson Xavier NX 対応
    - MotionJPEG ハードウェアデコーダ対応
    - H.264 / VP9 ハードウェアエンコーダ対応
    - H.264 / VP9 ハードウェアデコーダ対応
- 統計機能
    - Sora のシグナリング経由での出力
    - Ayame のシグナリング 経由での出力
- NVIDIA VIDEO CODEC SDK
    - H.264 ハードウェアエンコーダ対応
        - Ubuntu 18.04
        - Windows 10
    - VP8 / VP9 / H.264 ハードウェアデコーダ対応
        - Ubuntu 18.04
        - Windows 10
    - MotionJPEG ハードウェアデコーダ対応
        - Ubuntu 18.04
        - Windows 10
- Simulcast 対応
- Intel Media SDK 対応
    - VP8 / VP9 / H.264 ハードウェアエンコーダ対応
- Ubuntu 18.04 Raspberry Pi 対応
    - H.264 の HWA 対応を含む
- 録画対応
    - MP4 形式での出力
- ROS Melodic 対応
- ROS2 対応

## Momo についての電子書籍

Momo の原作者である @tnoho が書いた Momo のノウハウが沢山詰まった本が販売されています。

[WebRTCをブラウザ外で使ってブラウザでできることを増やしてみませんか?\(電子版\) \- でんでんらぼ \- BOOTH](https://tnoho.booth.pm/items/1572872)

## サポートについて

WebRTC Native Client Momo に関するバグ報告は GitHub Issues へお願いします。
それ以外については Discord へお願いします。

### バグ報告

GitHub Issues を利用する前に、まずは Discord へ質問をお願いします。

#### 理想的なバグ報告例

- [ローカルネット上のAyameを使用して接続した場合にSEGVが発生する · Issue \#100 · shiguredo/momo](https://github.com/shiguredo/momo/issues/100)

### Discord

- アドバイスします
- フィードバック歓迎します
- サポートしません

最新の状況などはこちらで共有しています。質問もこちらでのみ受け付けています。
GitHub Issues を利用する前に、まずは Discord へ質問をお願いします。

https://discord.gg/gmEuZye

### 有料サポートについて

WebRTC Native Client に対する有料でのサポート契約については WebRTC SFU Sora ライセンス契約をしているお客様が前提となります。

- Momo のテクニカルサポート
- OSS 公開前提での Momo への機能追加

## H.264 のライセンス費用について

H.264 ハードウェアエンコーダ **のみ** を利用している Momo 単体の配布においてはライセンス費用は不要ですが、
ハードウェアとセットで配布する場合はライセンス費用を支払う必要があります。

ただし、 Raspberry Pi においては H.264 のライセンスがハードウェア費用に含まれているため、
配布時にライセンス費用を支払う必要はありません。

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

- [カメラ無しのラズパイとmomoでテスト映像をWebRTCで配信する \- Qiita](https://qiita.com/tetsu_koba/items/789a19cb575953f41a1a)
- [libwebRTCでFakeVideoしてみた \- Qiita](https://qiita.com/aikw/items/efb3726eb808a913d9da)
- [ティアフォーにおける自動運転車両の遠隔監視・操縦システムのご紹介 \- Tier IV Tech Blog](https://tech.tier4.jp/entry/2019/01/22/170032)
- [Run WebRTC Native Client Momo on Raspberry Pi 3B \| SHIROKU\.NET](https://shiroku.net/robotics/run-webrtc-native-client-momo-on-raspberry-pi-3b/)
- [WebRTC Native Client Momo がスゲエ – papalagi\.org](https://papalagi.org/blog/archives/635)
- [ラズパイのmomoで音声付きでWebRTCで配信する \- Qiita](https://qiita.com/tetsu_koba/items/33e335fb46f26bbd3431)
- [ラズパイ\+momoでWebRTCで送信するときにマイクの代わりに音声ファイルを使用する \- Qiita](https://qiita.com/tetsu_koba/items/b887c1a0be9f26b795f2)
- [ラズパイのmomoでステレオ音声でWebRTCで配信する \- Qiita](https://qiita.com/tetsu_koba/items/6c07129caa5a08d5d172)

