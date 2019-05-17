# WebRTC Native Client Momo

[![GitHub tag (latest SemVer)](https://img.shields.io/github/tag/shiguredo/momo.svg)](https://github.com/shiguredo/momo)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

## WebRTC Native Client Momo について

WebRTC Native Client Momo は libwebrtc を利用しブラウザなしで様々な環境で動作する WebRTC ネイティブクライアントです。

特に Raspberry Pi 環境では Raspberry Pi の GPU に積まれている H.264 ハードウェアエンコーダー機能を利用することが可能です。

また [ROS](http://www.ros.org/) ノードとしても利用可能です。

## OpenMomo プロジェクトについて

OpenMomo は WebRTC Native Client Momo をオープンソースとして公開し、継続的に開発を行うことで、ブラウザ以外での WebRTC 利用を推進していくプロジェクトです。

詳細については下記をご確認ください。

[OpenMomo プロジェクト](https://gist.github.com/voluntas/51c67d0d8ce7af9f24655cee4d7dd253)

## develop ブランチにおける既知の問題

- 現在 macOS 版でカメラを認識できない事がわかっています
    - Unified Plan 対応にて修正予定です
- Raspberry Pi のハードウェアエンコーダを利用した場合に落ちる可能性があることがわかっています
    - MMAL 対応にて修正予定です

## 動作環境

- Ubuntu 18.04 x86_64
- Ubuntu 16.04 ARMv8
    - [ROCK64 –PINE64](https://www.pine64.org/?page_id=7147) で動作
- Raspbian Stretch ARMv7
    - Raspberry Pi 3 B/B+ で動作
- Raspbian Stretch ARMv6
    - Raspberry Pi Zero W/WH で動作
- Ubuntu 16.04 x86_64 ROS Kinetic で動作
    - http://wiki.ros.org/kinetic
- Ubuntu 16.04 ARMv7 ROS Kinetic で動作
    - Raspberry Pi 3 B+ で動作
- macOS 10.14 x86_64
    - バイナリの提供は行いません

## 使ってみる

Momo を使ってみたい人は [USE.md](doc/USE.md) をお読みください。

## ROS ノードとして使ってみる

Momo を ROS ノードとして使ってみたい人は [USE_ROS.md](doc/USE_ROS.md) をお読みください。

### ARM ROS 版

ARM ROS 版 Momo を ROS ノードとして使ってみたい人は [USE_ARM_ROS.md](doc/USE_ARM_ROS.md) をお読みください。

## ビルドに挑戦する

Linux 版 Momo のビルドに挑戦したい人は [BUILD.md](doc/BUILD.md) をお読みください。

### macOS 版 Momo のビルドに挑戦する

macOS 版 Momo のビルドに挑戦したい人は [BUILD_MACOS.md](doc/BUILD_MACOS.md) をお読みください。

### Windows 版 Momo のビルドに挑戦する

**準備中です**

## Pull-Request について

Momo はオープンソースソフトウェアですが、開発についてはオープンではありません。
そのため Pull-Request を頂いても採用できない場合があります。

まずは Discord にて気軽に話しかけてもらえれば。

## ライセンス

Apache License 2.0

```
Copyright 2018-2019, Shiguredo Inc, tnoho and melpon

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

WebRTC Native Client に関するバグ報告は GitHub Issues へお願いします。それ以外については Discord へお願いします。

### バグ報告

https://github.com/shiguredo/momo/issues

### Discord

ベストエフォートで運用しています。

https://discord.gg/gmEuZye

### 有料サポートについて

WebRTC Native Client に対する有料でのサポート契約については WebRTC SFU Sora ライセンス契約をしているお客様が前提となります。

## H.264 のライセンス費用について

- Raspberry Pi のハードウェアエンコーダのライセンス費用は Raspberry Pi の価格に含まれています
    - https://www.raspberrypi.org/forums/viewtopic.php?t=200855

## 利用例

- [libwebRTCでFakeVideoしてみた \- Qiita](https://qiita.com/aikw/items/efb3726eb808a913d9da)
- [ティアフォーにおける自動運転車両の遠隔監視・操縦システムのご紹介 \- Tier IV Tech Blog](https://tech.tier4.jp/entry/2019/01/22/170032)
