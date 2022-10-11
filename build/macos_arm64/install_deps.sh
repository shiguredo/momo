#!/bin/bash

cd "`dirname $0`"

set -ex

ARCH_NAME="aarch64-apple-darwin"

SOURCE_DIR="`pwd`/_source"
BUILD_DIR="`pwd`/_build"
INSTALL_DIR="`pwd`/_install"
CACHE_DIR="`pwd`/../../_cache"

mkdir -p $SOURCE_DIR
mkdir -p $BUILD_DIR
mkdir -p $INSTALL_DIR
mkdir -p $CACHE_DIR

source ../../VERSION

if [ -z "$JOBS" ]; then
  JOBS=`sysctl -n hw.logicalcpu_max`
  if [ -z "$JOBS" ]; then
    JOBS=1
  fi
fi

# CLI11
CLI11_VERSION_FILE="$INSTALL_DIR/cli11.version"
CLI11_CHANGED=0
if [ ! -e $CLI11_VERSION_FILE -o "$CLI11_VERSION" != "`cat $CLI11_VERSION_FILE`" ]; then
  CLI11_CHANGED=1
fi

if [ $CLI11_CHANGED -eq 1 -o ! -e $INSTALL_DIR/CLI11/include/CLI/Version.hpp ]; then
  pushd $INSTALL_DIR
    rm -rf CLI11
    git clone --branch v$CLI11_VERSION --depth 1 https://github.com/CLIUtils/CLI11.git
  popd
fi
echo $CLI11_VERSION > $CLI11_VERSION_FILE

# WebRTC
WEBRTC_VERSION_FILE="$INSTALL_DIR/webrtc.version"
WEBRTC_CHANGED=0
if [ ! -e $WEBRTC_VERSION_FILE -o "$WEBRTC_BUILD_VERSION" != "`cat $WEBRTC_VERSION_FILE`" ]; then
  WEBRTC_CHANGED=1
fi

if [ $WEBRTC_CHANGED -eq 1 -o ! -e $INSTALL_DIR/webrtc/lib/libwebrtc.a ]; then
  rm -rf $INSTALL_DIR/webrtc
  ../../script/get_webrtc.sh $WEBRTC_BUILD_VERSION macos_arm64 $INSTALL_DIR $SOURCE_DIR
fi
echo $WEBRTC_BUILD_VERSION > $WEBRTC_VERSION_FILE

# LLVM
if [ ! -e $INSTALL_DIR/llvm/clang/bin/clang++ ]; then
  rm -rf $INSTALL_DIR/llvm
  ../../script/get_llvm.sh $INSTALL_DIR/webrtc $INSTALL_DIR
fi

# Boost
BOOST_VERSION_FILE="$INSTALL_DIR/boost.version"
BOOST_CHANGED=0
if [ ! -e $BOOST_VERSION_FILE -o "$BOOST_VERSION" != "`cat $BOOST_VERSION_FILE`" ]; then
  BOOST_CHANGED=1
fi

CLANG="`xcodebuild -find clang`"
CLANGPP="`xcodebuild -find clang++`"

if [ $BOOST_CHANGED -eq 1 -o ! -e $INSTALL_DIR/boost/lib/libboost_filesystem.a ]; then
  rm -rf $SOURCE_DIR/boost
  rm -rf $BUILD_DIR/boost
  rm -rf $INSTALL_DIR/boost
  ../../script/setup_boost.sh $BOOST_VERSION $SOURCE_DIR/boost $CACHE_DIR/boost
  pushd $SOURCE_DIR/boost/source
    echo "using clang : : $CLANGPP : ;" > project-config.jam
    SYSROOT="`xcrun --sdk macosx --show-sdk-path`"
    ./b2 \
      cxxstd=17 \
      cflags=" \
        -target $ARCH_NAME \
        -mmacosx-version-min=11.0 \
        --sysroot=$SYSROOT \
      " \
      cxxflags=" \
        -target $ARCH_NAME \
        -mmacosx-version-min=11.0 \
        --sysroot=$SYSROOT \
      " \
      toolset=clang \
      visibility=hidden \
      link=static \
      variant=release \
      install \
      -d+0 \
      -j$JOBS \
      --build-dir=$BUILD_DIR/boost \
      --prefix=$INSTALL_DIR/boost \
      --ignore-site-config \
      --with-filesystem \
      --with-json
  popd
fi
echo $BOOST_VERSION > $BOOST_VERSION_FILE

# SDL2
SDL2_VERSION_FILE="$INSTALL_DIR/sdl2.version"
SDL2_CHANGED=0
if [ ! -e $SDL2_VERSION_FILE -o "$SDL2_VERSION" != "`cat $SDL2_VERSION_FILE`" ]; then
  SDL2_CHANGED=1
fi

if [ $SDL2_CHANGED -eq 1 -o ! -e $INSTALL_DIR/SDL2/lib/libSDL2.a ]; then
  rm -rf $SOURCE_DIR/SDL2
  rm -rf $BUILD_DIR/SDL2
  rm -rf $INSTALL_DIR/SDL2
  mkdir -p $SOURCE_DIR/SDL2
  mkdir -p $BUILD_DIR/SDL2
  ../../script/setup_sdl2.sh $SDL2_VERSION $SOURCE_DIR/SDL2
  pushd $BUILD_DIR/SDL2
    SYSROOT="`xcrun --sdk macosx --show-sdk-path`"
    # システムでインストール済みかによって ON/OFF が切り替わってしまうため、
    # どの環境でも同じようにインストールされるようにするため全部 ON/OFF を明示的に指定する
    cmake $SOURCE_DIR/SDL2/source \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=$INSTALL_DIR/SDL2 \
      -DBUILD_SHARED_LIBS=OFF \
      -DCMAKE_SYSTEM_PROCESSOR=arm64 \
      -DCMAKE_OSX_ARCHITECTURES=arm64 \
      -DCMAKE_C_COMPILER=clang \
      -DCMAKE_C_COMPILER_TARGET=aarch64-apple-darwin \
      -DCMAKE_CXX_COMPILER=clang++ \
      -DCMAKE_CXX_COMPILER_TARGET=aarch64-apple-darwin \
      -DCMAKE_SYSROOT=$SYSROOT \
      -DSDL_ATOMIC=OFF \
      -DSDL_AUDIO=OFF \
      -DSDL_VIDEO=ON \
      -DSDL_RENDER=ON \
      -DSDL_EVENTS=ON \
      -DSDL_JOYSTICK=ON \
      -DSDL_HAPTIC=ON \
      -DSDL_POWER=ON \
      -DSDL_THREADS=ON \
      -DSDL_TIMERS=OFF \
      -DSDL_FILE=OFF \
      -DSDL_LOADSO=ON \
      -DSDL_CPUINFO=OFF \
      -DSDL_FILESYSTEM=OFF \
      -DSDL_SENSOR=ON \
      -DSDL_OPENGL=ON \
      -DSDL_OPENGLES=ON \
      -DSDL_RPI=OFF \
      -DSDL_WAYLAND=OFF \
      -DSDL_X11=OFF \
      -DSDL_VULKAN=OFF \
      -DSDL_VIVANTE=OFF \
      -DSDL_COCOA=ON \
      -DSDL_METAL=ON \
      -DSDL_KMSDRM=OFF
    cmake --build . --config Release -j$JOBS
    cmake --install . --config Release
  popd
fi
echo $SDL2_VERSION > $SDL2_VERSION_FILE
