# Momo をビルドする

まずは Momo のリポジトリをダウンロードします。

```bash
git clone https://github.com/shiguredo/momo.git
```

## 事前準備 (Windows)

Windows 向けの Momo をビルドするには、以下の OS とアプリケーションが必要になります。

- 最新バージョンの Windows
- [Visual Studio 2022](https://visualstudio.microsoft.com/ja/downloads/) (どのエディションでも化）
  - C++ に関するコンポーネントを入れておいて下さい。特に MSVC, MSBuild は必須です。
- [Python](https://www.python.org/downloads/)
  - 最新バージョンをインストールして下さい。

## 事前準備 (macOS)

macOS 向けの Momo をビルドするには、以下の OS とアプリケーションが必要になります。

- 最新バージョンの macOS
- Xcode

Xcode に関しては、最低１回は単体で起動してライセンスに同意しておく必要があります。

## 事前準備 (Ubuntu)

Ubuntu、Raspberry OS、Jetson 向けの Momo をビルドするには、以下の OS が必要になります。

- Ubuntu 22.04 LTS

また、CUDA や Clang など、追加でいくつかのパッケージを入れておく必要があります。

必要なパッケージの詳細は [build.yml](../.github/workflows/build.yml) を参照してください。

## ビルドする

`python3 run.py build <target>` というコマンドで、各ターゲット向けの Momo を生成できます。

```bash
# このコマンドは Windows 上でしか動作しません。
# また、コマンドプロンプトや PowerShell 上で実行して下さい。
# Git Bash や Cygwin などのシェル上では動作しません。
python3 run.py build windows_x86_64

# このコマンドは macOS 上でしか動作しません
python3 run.py build macos_arm64

# このコマンドは Ubuntu 上でしか動作しません
python3 run.py build raspberry-pi-os_armv8
python3 run.py build ubuntu-22.04_x86_64
python3 run.py build ubuntu-22.04_jetson
```

生成された Momo の実行バイナリは `_build/<target>/release/momo` ディレクトリにあります。

## パッケージを作成する

ビルド時に `--package` オプションを指定することで、各ターゲット向けのパッケージを生成できます。

```bash
# windows_x86_64 の部分はビルドするターゲットに合わせて変更してください。
python3 run.py build windows_x86_64 --package
```

生成されたパッケージは `_package/<target>/release` ディレクトリに `momo-2023.1.0_raspberry-pi-os_armv8.tar.gz` のような名前で保存されています。

## パッケージ解凍後の構成

```console
$ tree
momo
├── LICENSE
├── NOTICE
├── html
│   ├── test.html
│   └── webrtc.js
└── momo
```
