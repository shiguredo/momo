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

- Windows 11 x86_64
- Ubuntu 24.04 x86_64
- Ubuntu 22.04 x86_64

## Windows 10 での利用方法

### ドライバーのインストール

Windows 11 ではインストール作業は必要ありません。

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

## Ubuntu 20.04、 Ubuntu 22.04 での利用方法

### ドライバーのインストール∂

- Ubuntu の最新化を実行します
  - `sudo apt-get update`
  - `sudo apt-get upgrade`
- ドライバー確認ツールをインストールします
  - `sudo apt-get install vainfo`
- ドライバーをインストールします。
  - 以下のコマンドのいずれでも問題ありません。フル機能版は `intel-media-va-driver-non-free` でコア機能版は `intel-media-va-driver` になります。
    - `sudo apt-get install intel-media-va-driver-non-free`
    - `sudo apt-get install intel-media-va-driver`
- 関連ライブラリをインストールします
  - `sudo apt install libmfx1`

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
