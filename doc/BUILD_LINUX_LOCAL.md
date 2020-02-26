# Linux 版 Momo をローカルでビルドする

Momo や WebRTC をいろいろと弄って試すために、Linux のローカルで手動ビルドしたい人向けの内容です。

Docker を使って簡単にビルドする方法については [Linux 版 Momo をビルドする](BUILD_LINUX.md) を参照してください。

## WebRTC をカスタマイズしない場合

WebRTC 本体をカスタマイズ（パッチを当てたりなど）をしない場合、[shiguredo-webrtc-build/webrtc-build](https://github.com/shiguredo-webrtc-build/webrtc-build) にある事前にビルドされた WebRTC を利用できるので、環境構築にそこまで時間が掛かりません

この場合は Dockerfile でやっていた内容をローカルに構築するだけになります。
例えば Ubuntu 18.04 で x86_64 向けのビルドを行う場合は [build/ubuntu-18.04_x86_64/Dockerfile](/build/ubuntu-18.04_x86_64/Dockerfile) を参照して構築スクリプトを記述していきます。

また、CMake での構築は `MOMO_PACKAGE_NAME` が利用できないので、それぞれの設定を自分の手で設定する必要があります。

以下は参考用の構築スクリプトになります。

このスクリプトは Ubuntu 18.04 で x86_64 向けの Momo バイナリを生成します。

`build/local/install_deps_local.sh`:

```bash
#!/bin/bash

cd "`dirname $0`"

set -ex

SOURCE_DIR="`pwd`/_source"
BUILD_DIR="`pwd`/_build"
INSTALL_DIR="`pwd`/_install"

mkdir -p $SOURCE_DIR
mkdir -p $BUILD_DIR
mkdir -p $INSTALL_DIR

source ../../VERSION

if [ -z "$JOBS" ]; then
  JOBS=`nproc`
  if [ -z "$JOBS" ]; then
    JOBS=1
  fi
fi

# CLI11
if [ ! -e $INSTALL_DIR/CLI11/include/CLI/Version.hpp ]; then
  pushd $INSTALL_DIR
    rm -rf CLI11
    git clone --branch v$CLI11_VERSION --depth 1 https://github.com/CLIUtils/CLI11.git
  popd
fi

# nlohmann/json
if [ ! -e $INSTALL_DIR/json/include/nlohmann/json.hpp ]; then
  pushd $INSTALL_DIR
    rm -rf json
    git clone --branch v$JSON_VERSION --depth 1 https://github.com/nlohmann/json.git
  popd
fi

# WebRTC
if [ ! -e $INSTALL_DIR/webrtc/lib/libwebrtc.a ]; then
  rm -rf $INSTALL_DIR/webrtc
  ../../script/get_webrtc.sh $WEBRTC_BUILD_VERSION ubuntu-18.04_x86_64 $INSTALL_DIR
fi

# LLVM
if [ ! -e $INSTALL_DIR/llvm/clang/bin/clang++ ]; then
  rm -rf $INSTALL_DIR/llvm
  ../../script/get_llvm.sh $INSTALL_DIR/webrtc $INSTALL_DIR
fi

# Boost
if [ ! -e $INSTALL_DIR/boost/lib/libboost_filesystem.a ]; then
  rm -rf $SOURCE_DIR/boost
  rm -rf $BUILD_DIR/boost
  rm -rf $INSTALL_DIR/boost
  mkdir -p $SOURCE_DIR/boost
  ../../script/setup_boost.sh $BOOST_VERSION $SOURCE_DIR/boost
  pushd $SOURCE_DIR/boost/source
    echo "using clang : : $INSTALL_DIR/llvm/clang/bin/clang++ : ;" > project-config.jam
    ./b2 \
      cxxflags=" \
        -D_LIBCPP_ABI_UNSTABLE \
        -nostdinc++ \
        -isystem$INSTALL_DIR/llvm/libcxx/include \
      " \
      toolset=clang \
      visibility=global \
      target-os=linux \
      address-model=64 \
      link=static \
      variant=release \
      install \
      -j$JOBS \
      --build-dir=$BUILD_DIR/boost \
      --prefix=$INSTALL_DIR/boost \
      --ignore-site-config \
      --with-filesystem
  popd
fi

# SDL2
if [ ! -e $INSTALL_DIR/SDL2/lib/libSDL2.a ]; then
  rm -rf $SOURCE_DIR/SDL2
  rm -rf $BUILD_DIR/SDL2
  rm -rf $INSTALL_DIR/SDL2
  mkdir -p $SOURCE_DIR/SDL2
  mkdir -p $BUILD_DIR/SDL2
  ../../script/setup_sdl2.sh $SDL2_VERSION $SOURCE_DIR/SDL2
  pushd $BUILD_DIR/SDL2
    CC=$INSTALL_DIR/llvm/clang/bin/clang \
    CXX=$INSTALL_DIR/llvm/clang/bin/clang++ \
    cmake \
      -DCMAKE_BUILD_TYPE=Release \
      -DBUILD_SHARED_LIBS=OFF \
      -DCMAKE_INSTALL_PREFIX=$INSTALL_DIR/SDL2 \
      $SOURCE_DIR/SDL2/source

    cmake --build . -j$JOBS
    cmake --build . --target install
  popd
fi
```

`build/build_local.sh`:

```bash
#!/bin/bash

cd "`dirname $0`"

INSTALL_DIR="`pwd`/local/_install"

./local/install_deps_local.sh

source ../VERSION

mkdir -p ../_build/local
pushd ../_build/local
  cmake \
    -DCMAKE_BUILD_TYPE=Release \
    -DMOMO_VERSION="$MOMO_VERSION" \
    -DMOMO_COMMIT="$MOMO_COMMIT" \
    -DWEBRTC_BUILD_VERSION="$WEBRTC_BUILD_VERSION" \
    -DWEBRTC_READABLE_VERSION="$WEBRTC_READABLE_VERSION" \
    -DWEBRTC_COMMIT="$WEBRTC_COMMIT" \
    -DTARGET_OS="linux" \
    -DTARGET_OS_LINUX="ubuntu-18.04" \
    -DTARGET_ARCH="x86_64" \
    -DUSE_SDL2=ON \
    -DBOOST_ROOT_DIR=$INSTALL_DIR/boost \
    -DJSON_ROOT_DIR=$INSTALL_DIR/json \
    -DCLI11_ROOT_DIR=$INSTALL_DIR/CLI11 \
    -DSDL2_ROOT_DIR=$INSTALL_DIR/SDL2 \
    -DWEBRTC_INCLUDE_DIR=$INSTALL_DIR/webrtc/include \
    -DWEBRTC_LIBRARY_DIR=$INSTALL_DIR/webrtc/lib \
    -DCLANG_ROOT=$INSTALL_DIR/llvm/clang \
    -DUSE_LIBCXX=ON \
    -DLIBCXX_INCLUDE_DIR=$INSTALL_DIR/llvm/libcxx/include \
    ../..

  cmake --build . -j$JOBS
popd
```

`./build/build_local.sh` を実行すると、WebRTC, Boost, SDL といった依存ライブラリを `build/local/_install` 以下にインストールし、`_build/local` に momo バイナリを生成します。

注意点として、Momo のバージョンが上がるにつれて依存ライブラリや引数が増えていきますが、このスクリプトが更新されていない可能性があります。
その場合には Dockerfile や [CMakeLists.txt](/CMakeLists.txt) を参照してうまく動かして下さい。

## WebRTC をカスタマイズする場合

WebRTC をカスタマイズする場合、自前で WebRTC をビルドし、それを利用する必要があります。
そのため時間も掛かるし、ビルドの難易度も上がります。

ビルド環境やターゲットによってやり方が変わってくるので、リファレンスだけ示します。

- [Chromium のビルド方法](https://www.chromium.org/developers/how-tos/get-the-code)
- [shiguredo-webrtc-build/webrtc-build](https://github.com/shiguredo-webrtc-build/webrtc-build) リポジトリのビルドスクリプト
- shiguredo/momo の [build ディレクトリ](/build) 以下にある Dockerfile
- WebRTC をカスタマイズしない場合の参考用スクリプト

参考用に、Ubuntu 18.04 の x86_64 環境で WebRTC をローカルでビルドして Momo をビルドするスクリプトを以下に載せています。
ビルド環境やターゲットに合わせて変更してみて下さい。

```bash
# TODO(melpon): 用意する
```
