# Windows 版 Momo をビルドする

まずは Momo のリポジトリをダウンロードします。

```shell
$ git clone git@github.com:shiguredo/momo.git
```

### 事前準備

ビルドには以下のアプリケーションが必要になります。

- [Visual Studio 2019](https://visualstudio.microsoft.com/ja/downloads/) (どのエディションでも化）
  - C++ に関するコンポーネントを入れておいて下さい。特に MSVC, MSBuild は必須です。
- [CMake](https://cmake.org/download/)
  - バージョン 3.16 以上をインストールして下さい。

### ビルド方法

build ディレクトリにある ./build.bat を起動することで Momo の Windows 向けバイナリが生成されます。

```
.\build.bat
```

生成されたバイナリは `_build\windows\Release\momo.exe` にあります。

### 中間ファイルのクリーンアップ

ビルド中にできた中間ファイルを削除するには、次のように -clean 引数を指定することでクリーンアップできます。

```shell
.\build.bat -clean
```
