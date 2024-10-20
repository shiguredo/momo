# Momo で VPL を利用したハードウェアエンコーダー / デコーダーを利用する

VPL を利用して Intel Quick Sync Video の HWA 機能を使った Momo で HWA を利用することが可能になります。

このドキュメントでは VPL を使用するためのセットアップ方法を記載します。

## 既知の問題

現在 VPL で VP9 と AV1 を送信するとき、受信に参加したクライアントが受信できない問題があります。
`-vp9-encoder` と `--av1-encoder` で　`software` を指定することで回避できます。
詳細については https://github.com/shiguredo/momo/issues/357 をご確認ください。

## Intel Media SDK について

VPL の詳細については以下のリンクをご確認ください。

- デコーダーとエンコーダーの対応しているコーデックとチップセットの組み合わせ表
  - <https://github.com/intel/media-driver#decodingencoding-features>
- VPL の Github
  - <https://github.com/intel/libvpl>
- Intel 公式ドキュメント
  - <https://intel.github.io/libvpl/latest/index.html>

## 対応プラットフォーム

- Windows 11 x86_64
- Ubuntu 22.04 x86_64
- Ubuntu 24.04 x86_64

## Windows 11 での利用方法

### ドライバーのインストール

Windows 11 では Intel の公式サイトからドライバーをインストールすることで VPL を利用することができます。

- Intel の公式サイトからドライバーをダウンロードします。
  - Intel ドライバーおよびソフトウェアのダウンロード
    - <https://www.intel.co.jp/content/www/jp/ja/download-center/home.html>
- インストーラーに従ってインストールを行います。
- インストール後に再起動を行います。

### VPL が認識できているか確認

Momo を `--video-codec-engines` オプションを指定して実行することで利用可能なエンコーダーとデコーダー一覧が出力されます。 `Encoder` と `Decoder` に `oneVPL [vpl]` が表示されているコーデックで利用可能です。

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
    - oneVPL [vpl] (default)
    - Software [software]

AV1:
  Encoder:
    - Software [software] (default)
  Decoder:
    - Software [software] (default)

H264:
  Encoder:
    - oneVPL [vpl] (default)
  Decoder:
    - oneVPL [vpl] (default)
```

## Ubuntu での利用方法

Ubuntu の場合は 22.04 と 24.04 で利用方法が異なります。

### Ubuntu 22.04 での利用方法

#### Intel の apt リポジトリを追加

ランタイムのインストールには Intel の apt リポジトリを追加する必要があります。

```bash
wget -qO - https://repositories.intel.com/gpu/intel-graphics.key | \
  sudo gpg --dearmor --output /usr/share/keyrings/intel-graphics.gpg
echo "deb [arch=amd64,i386 signed-by=/usr/share/keyrings/intel-graphics.gpg] https://repositories.intel.com/gpu/ubuntu jammy client" | \
  sudo tee /etc/apt/sources.list.d/intel-gpu-jammy.list
sudo apt update
```

#### Intel 提供パッケージの最新化

Intel の apt リポジトリを追加することでインストール済みのパッケージも Intel から提供されている最新のものに更新できます。依存問題を起こさないため、ここで最新化を行なってください。

```bash
sudo apt upgrade
```

#### ドライバとライブラリのインストール

以下のように、ドライバとライブラリをインストールしてください。
intel-media-va-driver には無印と `non-free` 版がありますが、 `non-free` 版でしか動作しません。

```bash
sudo apt install -y intel-media-va-driver-non-free libmfxgen1
```

以上でインストールが完了します。

### Ubuntu 24.04 での利用方法

#### Intel の apt リポジトリを追加

ランタイムのインストールには Intel の apt リポジトリを追加する必要があります。

```bash

wget -qO - https://repositories.intel.com/gpu/intel-graphics.key | \
  sudo gpg --dearmor --output /usr/share/keyrings/intel-graphics.gpg
echo "deb [arch=amd64,i386 signed-by=/usr/share/keyrings/intel-graphics.gpg] https://repositories.intel.com/gpu/ubuntu noble client" | \
  sudo tee /etc/apt/sources.list.d/intel-gpu-noble.list
sudo apt update
```

#### ライブラリのインストール

以下の実行例のように、 libmfxgen1 をインストールしてください。

```bash
sudo apt install -y libmfxgen1
```

以上でインストールが完了します。

### VPL が認識できているか確認

Momo を `--video-codec-engines` オプションを指定して実行することで利用可能なエンコーダーとデコーダー一覧が出力されます。 `Encoder` と `Decoder` に `oneVPL [vpl]` が表示されているコーデックで利用可能です。

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
    - oneVPL [vpl] (default)
    - Software [software]

AV1:
  Encoder:
    - Software [software] (default)
  Decoder:
    - Software [software] (default)

H264:
  Encoder:
    - oneVPL [vpl] (default)
  Decoder:
    - oneVPL [vpl] (default)
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

## エンコーダーが複数ある場合

NVIDIA と共存させた環境の場合 INTEL と NVIDIA のエンコーダーが表示されます。
Momo では NVIDIA を優先して使用するようになっていますが `--h264-encoder` オプションを使用して `vpl` を指定することで oneVPL を使用することができます。

## VPL を認識できない場合

NVIDIA を利用している環境では VPL を認識できないことがあります。その場合は NVIDIA のドライバーを削除し Intel のグラフィックドライバーに切り替えると認識する場合があります。
