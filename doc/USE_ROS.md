# ROS 版 momo を使ってみる

- OS: Ubuntu 16.04

## ROS 環境を用意する

ROS 環境を用意しておきます。

- ROS Kinetic のインストール方法: http://wiki.ros.org/kinetic/Installation/Ubuntu


## momo の準備

### momo のダウンロード

https://github.com/shiguredo/momo/releases にて ROS 版 momo のバイナリをダウンロードしてください。

必要なライブラリをインストールしてご利用ください。

#### 解凍後の構成

```
$ tree
.
├── html
│   ├── p2p.html
│   └── webrtc.js
├── LICENSE
├── momo
└── NOTICE
```

#### ライブラリのインストール

次のパッケージをイストールします。

```
$ sudo apt -y install libnss3
```


## 実行する

momo を実行する前に下記のように rosrun を使用して Web カメラを起動しておきます。

事前に、apt で ros-kinetic-usb-cam をインストールした上で実行します。

```
$ rosrun usb_cam usb_cam_node _pixel_format:=mjpeg
```

### P2P で動作を確認する

- 変更可能なパラメータ
  - image
    - topic
  - _port
    - ポート番号  [0 - 65535]
- 実行例

```shell
$ ./momo  _use_p2p:=true \
          _compressed:=true \
          image:=/usb_cam/image_raw/compressed \
          _port:=8080
```

http://[momo の IP アドレス]:8080/html/p2p.html にアクセスしてください。

image は Web カメラから送られてくる画像データの topic を指定してください。

- 変更可能なパラメータ


### WebRTC SFU Sora で動作を確認する

**この機能を利用する場合は WebRTC SFU Sora のライセンス契約が必要です**

```shell
$ ./momo  _use_sora:=true \
          _auto:=true  \
          _compressed:=true \
          _port:=0 \
          _SIGNALING_URL:="wss://example.com/signaling" \
          _CHANNEL_ID:="sora" \
          _video_codec:=H264 \
          _log_level:=5 \
          _video_bitrate:=300 \
          image:=/usb_cam/image_raw/compressed
```

image は Web カメラから送られてくる画像データの topic を指定してください。


- 変更可能なパラメータ
  - _SIGNALING_URL
    - シグナリング URL
  - _CHANNEL_ID
    - チャネル ID
  - image
    - topic
  - _video_codec
    - ビデオコーデック  [VP8,VP9,H264]
  - _video_bitrate
    - ビデオビットレート  [1 - 30000]
  - _log_level
    - ログレベル  [0 - 5]
