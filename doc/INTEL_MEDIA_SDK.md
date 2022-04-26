# INTEL MEDIA SDK を使用する

## 対応プラットフォーム

- Windows 10 x86_64
- Ubuntu 20.04 x86_64

## Windows10 での利用方法

### ドライバーのインストール

[インテル® グラフィックス - Windows* DCH Driver](https://www.intel.co.jp/content/www/jp/ja/download/19344/intel-graphics-windows-dch-drivers.html) をダウンロードしてインストールします。

インストールが完了すると INTEL MEDIA SDK を利用する準備が完了します。

## INTEL MEDIA SDK の動作確認

`--video-codec-engines ` を指定して Momo を実行することで利用可能なエンコーダーとデコーダー一覧が出力されます。

PowerShell での実行例：
```
.\momo.exe --video-codec-engines
```

実行結果例：
```
.\momo.exe --video-codec-engines
VP8:
  Encoder:
    - Software [software] (default)
  Decoder:
    - Software [software] (default)

VP9:
  Encoder:
    - Software [software] (default)
  Decoder:
    - Intel Media SDK [intel] (default)
    - Software [software]

AV1:
  Encoder:
    - Software [software] (default)
  Decoder:
    - Software [software] (default)

H264:
  Encoder:
    - Intel Media SDK [intel] (default)
  Decoder:
    - Intel Media SDK [intel] (default)
```

## Ubuntu20.04 での利用方法

### ドライバーのインストール

- Ubuntu の最新化を実行します
    - `sudo apt-get update`
    - `sudo apt-get upgrade`
- ドライバー確認ツールをインストールします
    - `sudo apt-get install vainfo`
- INTEL MEDIA ドライバー ( `intel-media-va-driver` または `intel-media-va-driver-non-free` ) をインストールします。
    - `sudo apt-get install intel-media-va-driver-non-free`
    または `sudo apt-get install intel-media-va-driver`
- 関連ライブラリをインストールします
    - `sudo apt install libmfx1`

以上でインストールが完了します。

## Momo での動作確認

`--video-codec-engines ` を指定して Momo を実行することで利用可能なエンコーダーとデコーダー一覧が出力されます。

実行コマンド例：
```
./momo --video-codec-engines
```

実行結果例：
```
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
    - Intel Media SDK [intel] (default)
    - Software [software]

AV1:
  Encoder:
    - Software [software] (default)
  Decoder:
    - Software [software] (default)

H264:
  Encoder:
    - Intel Media SDK [intel] (default)
  Decoder:
    - Intel Media SDK [intel] (default)
```

### エンコーダーが複数ある場合

NVIDIA と共存させた環境の場合 INTEL と NVIDIA のエンコーダーが表示されます。
Momo では NVIDIA を優先して使用するようになっていますが `--h264-encoder` オプションを使用して INTEL を指定することで INTEL MEDIA SDK を使用することができます。

### うまく INTEL MEDIA SDK をインストールできない場合

NVIDIA を利用している環境で INTEL MEDIA SDK をうまくインストールできないことがあります。
その時は一度 NVIDIA のドライバーを削除してからインストールするとインストール可能になる場合があります。
