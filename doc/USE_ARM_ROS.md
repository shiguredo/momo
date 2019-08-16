# ARM ROS 版 Momo を使ってみる

動作確認済み環境は下記の通り

- Board: Raspberry Pi 3 Model B+
- OS: Ubuntu 16.04
- ROS: Kinetic

検証にあたっては、こちらのブログを参考に環境を構築しました。謝意を表します。
[RaspberryPi 3B+でUbuntu 16.04を起動させる方法](https://www.asrobot.me/entry/2018/07/11/001603/)

## Momo の準備

### Momo のビルド

[BUILD.md](./BUILD.md)を確認してUbuntu 16.04 (armv7) 向け ROS 対応パッケージを作成してください。

```shell
$ make ubuntu-16.04_armv7_ros.package
```

#### Raspberry Pi への配置

下記のような構成で Raspberry Pi 内にビルドした Momo を配置します。

```
$ tree
.
├── html
│   ├── test.html
│   └── webrtc.js
└── ビルドした momo
```


Momo のパッケージを https://github.com/shiguredo/momo/releases からダウンロードした場合は、パッケージを展開すると既に上記と同じ構成で配置されています。

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


#### ライブラリのインストール

次のパッケージをイストールします。

```
$ sudo apt -y install libnss3 libasound2 gstreamer1.0-alsa
```

また H264 ハードウェアエンコーダに対応するため、下記のリポジトリを追加し、パッケージをインストールします。

```
$ sudo add-apt-repository ppa:ubuntu-raspi2/ppa
$ sudo apt-get update
$ sudo apt-get install libraspberrypi-bin libraspberrypi-dev
```

Raspberry Pi の場合はハードウェアエンコーダを利用することで、非常に少ない CPU 消費で配信を行うことが可能です。

## 実行する

Momo を実行する前に下記のように rosrun を使用して Web カメラ、マイクを起動しておきます。

事前に、apt で ros-kinetic-usb-cam, ros-kinetic-audio-common をインストールした上で実行します。

```
$ rosrun usb_cam usb_cam_node
```

Raspberry Pi の場合は非常にリソースが限られていますので、Image topic は無圧縮での利用をお勧めします。

```
$ rosrun audio_capture audio_capture _format:=wave _channels=1 _same_rate:=16000
```

### Test で動作を確認する

- 実行例

```shell
$ ./momo  _use_test:=true \
          _compressed:=false \
          image:=/usb_cam/image_raw \
          audio:=/audio \
          _audio_topic_ch:=1 \
          _audio_topic_rate:=16000 \
          _port:=8080
```

http://[momo の IP アドレス]:8080/html/test.html にアクセスしてください。

image には Web カメラから送られてくる画像データの topic を指定してください。
audio にはマイクから送られてくる音声データの topic を指定してください。

- 変更可能なパラメータ
  - image
    - topic
  - _compressed
    - JPEG 圧縮済みイメージ topic か  [true,false]
  - _port
    - ポート番号  [0 - 65535]
  - _log_level
    - ログレベル  [0 - 5]
  - audio
    - audio topic
  - _audio_topic_ch
    - チャネル数    [1]
  - _audio_topic_rate
    - サンプリングレート


### WebRTC SFU Sora で動作を確認する

**この機能を利用する場合は WebRTC SFU Sora のライセンス契約が必要です**

```shell
$ ./momo  _use_sora:=true \
          _auto:=true  \
          _compressed:=false \
          _port:=0 \
          _SIGNALING_URL:="wss://example.com/signaling" \
          _CHANNEL_ID:="sora" \
          _video_codec:=H264 \
          _log_level:=5 \
          _video_bitrate:=300 \
          image:=/usb_cam/image_raw \
          audio:=/audio \
          _audio_topic_ch:=1 \
          _audio_topic_rate:=16000
```

image には Web カメラから送られてくる画像データの topic を指定してください。
audio にはマイクから送られてくる音声データの topic を指定してください。


- 変更可能なパラメータ
  - _SIGNALING_URL
    - シグナリング URL
  - _CHANNEL_ID
    - チャネル ID
  - image
    - topic
  - _compressed
    - JPEG 圧縮済みイメージ topic か  [true,false]
  - _video_codec
    - ビデオコーデック  [H264,VP8,VP9]
  - _video_bitrate
    - ビデオビットレート  [1 - 30000]
  - _log_level
    - ログレベル  [0 - 5]
  - audio
    - audio topic
  - _audio_topic_ch
    - チャネル数    [1]
  - _audio_topic_rate
    - サンプリングレート
