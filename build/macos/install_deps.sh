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
  JOBS=`sysctl -n hw.logicalcpu_max`
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
  ../../script/get_webrtc.sh $WEBRTC_BUILD_VERSION macos $INSTALL_DIR
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
    SYSROOT="`xcrun --sdk macosx --show-sdk-path`"
    ./b2 \
      cflags=" \
        --sysroot=$SYSROOT \
      " \
      cxxflags=" \
        -isystem $INSTALL_DIR/llvm/libcxx/include \
        -nostdinc++ \
        --sysroot=$SYSROOT \
      " \
      toolset=clang \
      visibility=hidden \
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
    # SDL2 の CMakeLists.txt は Metal をサポートしてくれてないので、configure でビルドする
    # ref: https://bugzilla.libsdl.org/show_bug.cgi?id=4617
    SYSROOT="`xcrun --sdk macosx --show-sdk-path`"
    CC="$INSTALL_DIR/llvm/clang/bin/clang --sysroot=$SYSROOT" \
      CXX="$INSTALL_DIR/llvm/clang/bin/clang++ --sysroot=$SYSROOT -nostdinc++" \
      $SOURCE_DIR/SDL2/source/configure --disable-shared --prefix=$INSTALL_DIR/SDL2
    make -j$JOBS
    make install
  popd
fi
