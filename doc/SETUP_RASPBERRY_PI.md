# Raspberry Pi (Raspberry Pi OS) で Momo を使ってみる

## 注意

Raspberry Pi OS のレガシー版には対応していません。最新版の Raspberry Pi OS (64 bit) を利用してください。

## Raspberry Pi 向けのバイナリは以下にて提供しています

<https://github.com/shiguredo/momo/releases> にて最新版のバイナリをダウンロードしてください。

- Raspberry Pi OS 64 bit を利用する場合は、 `momo-<VERSION>_raspberry-pi-os_armv8.tar.gz` を利用してください

## ダウンロードしたパッケージ、解凍後の構成

解凍後の構成は [BUILD.md のパッケージ解凍後の構成](BUILD.md#パッケージ解凍後の構成) を参照してください。

## 準備

### パッケージのインストール

下記を実行してください。

```bash
sudo apt-get update
sudo apt-get upgrade
sudo apt-get install libnspr4 libnss3
```

解凍したディレクトリで `ldd ./momo | grep not` を実行し、不足している共有ライブラリを確認してください。不足があれば、対応するパッケージをインストールしてください。

CSI カメラを利用する場合は libcamera が必要です。パッケージに同梱されている `libcamerac.so` が libcamera に依存します。依存する SONAME はリリースごとに変わるため、特定バージョンのパッケージ名を固定しないでください。

例: GitHub Releases の 2025.1.3 (`momo-2025.1.3_raspberry-pi-os_armv8.tar.gz`) では、`libcamerac.so` が `libcamera.so.0.7` と `libcamera-base.so.0.7` に依存します。この場合は次をインストールします。

```bash
sudo apt-get install libcamera0.7
```

利用するバイナリが 2025.1.3 でない場合は、そのバイナリで `ldd ./momo | grep not` を実行し、表示されたライブラリに合わせてパッケージを選んでください。

#### Raspberry Pi OS Lite を利用する場合

Raspberry Pi OS Lite では映像に関するパッケージが入っていないことがあります。同様に `ldd ./momo | grep not` で不足を確認してください。

下記に実行する一例を示します。

```bash
sudo apt-get install libxtst6
sudo apt-get install libegl1-mesa-dev
sudo apt-get install libgles2-mesa
```

## CSI カメラ (libcamera)

USB カメラを利用する場合、この節の手順は不要です。

Raspberry Pi OS bookworm 以降では従来のカメラシステムは利用できません。CSI カメラ (Raspberry Pi 専用カメラ) は `--use-libcamera` を指定してください。詳細は [LIBCAMERA.md](LIBCAMERA.md) を確認してください。

```bash
./momo --use-libcamera --no-audio-device p2p
```

CSI カメラの性能を上げる場合は `--use-libcamera-native` を検討してください。H.264 かつサイマルキャストが無効のときだけ有効です。制約と使い方は [LIBCAMERA.md](LIBCAMERA.md) を確認してください。

```bash
./momo --use-libcamera --use-libcamera-native --no-audio-device p2p
```

`--force-i420` と `--hw-mjpeg-decoder` は V4L2 キャプチャー向けのオプションです。`--use-libcamera` 指定時に `--force-i420` を付けてもキャプチャー経路には効果がありません。`--hw-mjpeg-decoder true` を同時指定しても libcamera の MJPEG ハードウェアデコードや V4L2 リサイズは走らず、ソフトウェアエンコーダーを使わない設定だけが有効になります。CSI カメラの性能改善には使いません。

## USB カメラ (V4L2)

USB カメラは V4L2 キャプチャーを使います。`--use-libcamera` は指定しないでください。

### `--hw-mjpeg-decoder`

Raspberry Pi OS 向けバイナリでは、`--hw-mjpeg-decoder` のデフォルトは `false` です。一部の MJPEG に対応した USB カメラでは、`true` を指定すると MJPEG のハードウェアデコードとハードウェアによるビデオのリサイズを行います。この指定はソフトウェアエンコーダーを使わない設定にもなります。

```bash
./momo --hw-mjpeg-decoder true --no-audio-device p2p
```

USB カメラ利用時にフレームレートを出したい場合は `--hw-mjpeg-decoder` を指定しない方法もあります。ただし CPU 使用率は上がります。

CPU 使用率を抑えつつフレームレートを出したい場合は、`/boot/firmware/config.txt` の末尾に下記を追記して `--hw-mjpeg-decoder` を指定することで改善することがあります。

```text
gpu_mem=256
force_turbo=1
avoid_warnings=2
```

この設定であれば HD は 30 fps、FHD では 15 fps 程度の性能を発揮します。

### `--force-i420`

`--force-i420` は V4L2 キャプチャーでピクセルフォーマットを I420 に固定します。利用できない場合は起動に失敗します。`--use-libcamera` 指定時には効果がありません。

USB カメラではフレームレートが落ちることがあるため、使わないでください。

```bash
./momo --force-i420 --no-audio-device p2p
```

## 使ってみる

[USE_P2P.md](USE_P2P.md) を確認してください。

## ビデオデバイスの指定

ビデオデバイスの指定については [LINUX_VIDEO_DEVICE.md](LINUX_VIDEO_DEVICE.md) を確認してください。
