# Momo で VPL を利用したハードウェアエンコーダー / デコーダーを利用する

VPL を利用して Intel Quick Sync Video の HWA 機能を使った Momo で HWA を利用することが可能になります。

このドキュメントでは VPL を使用するためのセットアップ方法を記載します。

## Intel Media SDK について

VPL の詳細については以下のリンクをご確認ください。

- デコーダーとエンコーダーの対応しているコーデックとチップセットの組み合わせ表
  - <https://github.com/intel/media-driver#decodingencoding-features>
- VPL の Github
  - <https://github.com/intel/libvpl>
- Intel 公式ドキュメント
  - <https://intel.github.io/libvpl/latest/index.html>

## 対応プラットフォーム

- Ubuntu 24.04 x86_64
- Ubuntu 22.04 x86_64
- Windows 11 x86_64

## Windows 11 での利用方法

### ドライバーのインストール

Windows 11 では Intel の公式サイトからドライバーをインストールすることで VPL を利用することができます。

- Intel の公式サイトからドライバーをダウンロードします。
  - Intel ドライバーおよびソフトウェアのダウンロード
    - <https://www.intel.co.jp/content/www/jp/ja/download-center/home.html>
- インストーラーに従ってインストールを行います。
- インストール後に再起動を行います。

### VPL が認識できているか確認

Momo を `--video-codec-engines` オプションを指定して実行することで利用可能なエンコーダーとデコーダー一覧が出力されます。 `Encoder` と `Decoder` に `Intel VPL [vpl]` が表示されているコーデックで利用可能です。

PowerShell での実行コマンド例：

```powershell
.\momo.exe --video-codec-engines
```

PowerShell での実行結果例：

```powershell
> .\momo.exe --video-codec-engines
VP8:
  Encoder:
    - Software [software] (default)
  Decoder:
    - Software [software] (default)

VP9:
  Encoder:
    - Software [software] (default)
  Decoder:
    - Intel VPL [vpl] (default)
    - Software [software]

AV1:
  Encoder:
    - Software [software] (default)
  Decoder:
    - Software [software] (default)

H264:
  Encoder:
    - Intel VPL [vpl] (default)
  Decoder:
    - Intel VPL [vpl] (default)
```

## Ubuntu での利用方法

### Ubuntu 24.04 での利用方法

ランタイムのインストールには Intel の apt リポジトリを追加する必要があります。

```bash
sudo apt update
sudo apt -y install wget gpg

# Intel の GPG キーをインストールする
wget -qO - https://repositories.intel.com/gpu/intel-graphics.key | sudo gpg --dearmor --output /usr/share/keyrings/intel-graphics.gpg
# Intel のリポジトリを追加する
echo "deb [arch=amd64,i386 signed-by=/usr/share/keyrings/intel-graphics.gpg] https://repositories.intel.com/gpu/ubuntu noble client" | sudo tee /etc/apt/sources.list.d/intel-gpu-noble.list

sudo apt update
# Sora Python SDK に必要なライブラリをインストールする
sudo apt -y install git libva2 libdrm2 make build-essential libx11-dev
# Intel VPL に必要なライブラリをインストールする
sudo apt -y install intel-media-va-driver-non-free libmfx1 libmfx-gen1 libvpl2 libvpl-tools libva-glx2 va-driver-all vainfo

# sudo で vainfo が実行できるか確認する
sudo vainfo --display drm --device /dev/dri/renderD128

# udev のルールを追加する
sudo echo 'KERNEL=="render*" GROUP="render", MODE="0666"' > /etc/udev/rules.d/99-vpl.rules
# 再起動する
sudo reboot

# vainfo が sudo なしで実行できるか確認する
vainfo --display drm --device /dev/dri/renderD128
```

### Ubuntu 22.04 での利用方法

ランタイムのインストールには Intel の apt リポジトリを追加する必要があります。

```bash
sudo apt update
sudo apt -y install wget gpg

# Intel の GPG キーをインストールする
wget -qO - https://repositories.intel.com/gpu/intel-graphics.key | sudo gpg --dearmor --output /usr/share/keyrings/intel-graphics.gpg
# Intel のリポジトリを追加する
echo "deb [arch=amd64,i386 signed-by=/usr/share/keyrings/intel-graphics.gpg] https://repositories.intel.com/gpu/ubuntu jammy client" | sudo tee /etc/apt/sources.list.d/intel-gpu-jammy.list

sudo apt update
# Sora Python SDK に必要なライブラリをインストールする
sudo apt -y install git libva2 libdrm2 make build-essential libx11-dev
# Intel VPL に必要なライブラリをインストールする
sudo apt -y install intel-media-va-driver-non-free libmfx1 libmfx-gen1 libvpl2 libvpl-tools libva-glx2 va-driver-all vainfo

# sudo で vainfo が実行できるか確認する
sudo vainfo --display drm --device /dev/dri/renderD128

# udev のルールを追加する
sudo echo 'KERNEL=="render*" GROUP="render", MODE="0666"' > /etc/udev/rules.d/99-vpl.rules
# 再起動する
sudo reboot

# vainfo が sudo なしで実行できるか確認する
vainfo --display drm --device /dev/dri/renderD128
```

### VPL が認識できているか確認

Momo を `--video-codec-engines` オプションを指定して実行することで利用可能なエンコーダーとデコーダー一覧が出力されます。 `Encoder` と `Decoder` に `Intel VPL [vpl]` が表示されているコーデックで利用可能です。

実行コマンド例：

```bash
./momo --video-codec-engines
```

実行結果例：

```bash
$ ./momo --video-codec-engines
VP8:
  Encoder:
    - Software [software] (default)
  Decoder:
    - Software [software] (default)

VP9:
  Encoder:
    - Software [software] (default)
  Decoder:
    - Intel VPL [vpl] (default)
    - Software [software]

AV1:
  Encoder:
    - Software [software] (default)
  Decoder:
    - Software [software] (default)

H264:
  Encoder:
    - Intel VPL [vpl] (default)
  Decoder:
    - Intel VPL [vpl] (default)
```

## 動作確認ができたチップセット

現在動作確認ができているチップセットは以下になります。

- Intel(R) Core(TM) Ultra 5 Processor 125H
- Intel(R) Core(TM) i9-9980HK
- Intel(R) Core(TM) i7-1195G7
- Intel(R) Core(TM) i5-10210U
- Intel(R) Processor N97
- Intel(R) Processor N100
- Intel(R) Processor N95

## 動作が未確認のチップセット

- Intel(R) Processor N150

## エンコーダーが複数ある場合

NVIDIA と共存させた環境の場合 INTEL と NVIDIA のエンコーダーが表示されます。
Momo では NVIDIA を優先して使用するようになっていますが `--h264-encoder` オプションを使用して `vpl` を指定することで Intel VPL を使用することができます。

## VPL を認識できない場合

NVIDIA を利用している環境では VPL を認識できないことがあります。その場合は NVIDIA のドライバーを削除し Intel のグラフィックドライバーに切り替えると認識する場合があります。
