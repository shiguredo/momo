# Momo のビルドに挑戦する

ビルドにはマシンパワーにもよりますが、少なくとも 30 分以上かかり、さらに 20GB 以上のダウンロードが必要です。
そのため、覚悟を持って make コマンドを叩いてください。

Docker 18.09 以降が必要になりますので、事前にインストールしておいてください。 Windows の docker は未検証です。Linux 版、または macOS 版の Docker をご利用ください。
また、make コマンド実行時に NOMOUNT=1 オプションを指定することで、マウントを利用しないモードで docker container を動作させることができます。何らかの理由でマウントがうまく動作しない場合に使って下さい。

まずは momo のリポジトリをダウンロードします。git submodule を利用しているため --recursive を忘れないでください。

```shell
$ git clone --recursive git@github.com:shiguredo/momo.git
```

## Raspbian June 2018 (armv7) 向けバイナリを作成する

Raspberry Pi 3 B/B+ は実際は armv8 ですが 64 ビット機能が Raspbian では利用できないため、実質 armv7 相当のビルドになります。

build ディレクトリ以下で make raspbian-stretch_armv7 と打つことで Momo のバイナリが生成されます。

```shell
$ make raspbian-stretch_armv7
```

うまくいかない場合は `make clean && make raspbian-stretch_armv7` を試してみてください。それでもだめな場合は issues にお願いします。

## Raspbian June 2018 (armv6) 向けバイナリを作成する

build ディレクトリ以下で make raspbian-stretch_armv6 と打つことで Momo のバイナリが生成されます。

```shell
$ make raspbian-stretch_armv6
```

うまくいかない場合は `make clean && make raspbian-stretch_armv6` を試してみてください。それでもだめな場合は issues にお願いします。

## Ubuntu 16.04 (armv8) 向けバイナリを作成する

build ディレクトリ以下で make ubuntu-16.04_armv8 と打つことで Momo のバイナリが生成されます。

```shell
$ make ubuntu-16.04_armv8
```

うまくいかない場合は `make clean && make ubuntu-16.04_armv8` を試してみてください。それでもだめな場合は issues にお願いします。

## Ubuntu 18.04 (x86_64) 向けバイナリを作成する

build ディレクトリ以下で make ubuntu-18.04_x86_64 と打つことで Momo の Ubuntu 18.04 x86_64 向けバイナリが生成されます。

```shell
$ make ubuntu-18.04_x86_64
```

## macOS 10.14

### 事前確認

libwebrtc をビルドするためには、XCode をインストールし、最低１回は起動してライセンスに同意しておく必要があります。
現時点では、開発ツールのスタンドアローンインストール（`/Library/Developer/CommandLineTools` にインストールされるもの）はサポートされていません。
詳細は、次のリンク先をご覧ください。

https://bugs.chromium.org/p/chromium/issues/detail?id=729990#c1

開発マシンがどちらを使っているかは xcode-select --print-path で確かめることができます。
XCode のものが利用されている場合は、次のような出力になります。

```shell
$ xcode-select --print-path
/Applications/Xcode.app/Contents/Developer
```

ここでは AppStore から XCode をインストールしたケースを想定しています。
もし XCode の beta 版をインストールしている場合は、`/Applications/Xcode.app` の部分を適宜読み替えてください。

スタンドアローンインストールされたものが利用されている場合は、次のような出力になります。

```shell
$ xcode-select --print-path
/Library/Developer/CommandLineTools
```

この場合、次のコマンドを入力し、ビルド前に使用される CLI のパスを切り替えてください。

```shell
$ sudo xcode-select -s /Applications/Xcode.app
```

また、libwebrtc のビルド中にいくつかの Python スクリプトが呼び出されますが、そこでは Python 2 系が入っていることが期待されています。
macOS に同梱されている Python 以外の python を入れている場合は、バージョンを確認して、必要に応じてバージョンを切り替えておいてください。
例えば `pyenv` を利用している場合は、ビルド前に system を利用するように指定し、Python 2 が利用されること確認してください。

```shell
$ pyenv local system
$ python --version
Python 2.7.10
```

### ビルド方法

build ディレクトリ以下で make macos と打つことで Momo の macOS 向けバイナリが生成されます。

```shell
$ make macos
```

## Windows 10

**現在準備中です**
