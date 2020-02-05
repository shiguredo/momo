# ROS 版 Momo を使ってみる

- OS: Ubuntu 16.04

## ROS 環境を用意する

ROS 環境を用意しておきます。

- ROS Kinetic のインストール方法: http://wiki.ros.org/kinetic/Installation/Ubuntu

## Momo の準備

### Momo のビルド

[BUILD.md](./BUILD.md)を確認して Ubuntu 16.04 (x86_64) 向け ROS 対応バイナリをビルドしてください。

```shell
$ ./build.sh ubuntu-16.04_x86_64_ros
```

#### ライブラリのインストール

次のパッケージをイストールします。

```
$ sudo apt -y install libnss3 \
                      gstreamer1.0-alsa
```


## 実行する

Momo を実行する前に下記のように rosrun を使用して Web カメラ、マイクを起動しておきます。

事前に、apt で ros-kinetic-usb-cam, ros-kinetic-audio-common をインストールした上で実行します。

```
$ rosrun usb_cam usb_cam_node _pixel_format:=mjpeg
```

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
          _video_codec:=VP9 \
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
    - ビデオコーデック  [VP8,VP9]
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
