# Momo のビルドに挑戦する

ビルドにはマシンパワーにもよりますが、少なくとも 30 分以上かかり、さらに 20GB 以上のダウンロードが必要です。
そのため、覚悟を持って make コマンドを叩いてください。

まずは momo のリポジトリをダウンロードします。

```shell
$ git clone git@github.com:shiguredo/momo.git
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


## 中間ファイルのクリーンアップ

ビルド中にできた中間ファイルを削除するには、次のようにターゲットを指定して make _ターゲット_.clean を実行することでクリーンアップできます。例えば raspbian-stretch_armv7 をターゲットにしている場合は、build ディレクトリ以下で次のようクリーンアップします。

```shell
$ make raspbian-stretch_armv7.clean
```

