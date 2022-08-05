# macOS 版 Momo をビルドする

まずは Momo のリポジトリをダウンロードします。

```shell
$ git clone git@github.com:shiguredo/momo.git
```

## macOS 11

### 事前確認

Momo をビルドするためには、Xcode をインストールし、最低１回は起動してライセンスに同意しておく必要があります。
現時点では、開発ツールのスタンドアローンインストール（`/Library/Developer/CommandLineTools` にインストールされるもの）はサポートされていません。
詳細は、次のリンク先をご覧ください。

https://bugs.chromium.org/p/chromium/issues/detail?id=729990#c1

開発マシンがどちらを使っているかは xcode-select --print-path で確かめることができます。
Xcode のものが利用されている場合は、次のような出力になります。

```shell
$ xcode-select --print-path
/Applications/Xcode.app/Contents/Developer
```

ここでは AppStore から Xcode をインストールしたケースを想定しています。
もし Xcode の beta 版をインストールしている場合は、`/Applications/Xcode.app` の部分を適宜読み替えてください。

スタンドアローンインストールされたものが利用されている場合は、次のような出力になります。

```shell
$ xcode-select --print-path
/Library/Developer/CommandLineTools
```

この場合、次のコマンドを入力し、ビルド前に使用される CLI のパスを切り替えてください。

```shell
$ sudo xcode-select -s /Applications/Xcode.app
```

### CMake

ビルドには CMake 3.16 以上が必要になります。

Homebrew が入っている場合は `brew install cmake` するか、あるいは公式サイトからダウンロードして PATH を通しておいてください。

### ビルド方法

build ディレクトリ以下で ./build.sh macos_arm64 と打つことで Momo の macOS 向けバイナリが生成されます。

```shell
$ ./build.sh macos_arm64
```

## 中間ファイルのクリーンアップ

ビルド中にできた中間ファイルを削除するには、次のようにターゲットを指定して ./build.sh --clean macos_arm64 を実行することでクリーンアップできます。

```shell
$ ./build.sh --clean macos_arm64
```
