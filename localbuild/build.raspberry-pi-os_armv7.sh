#!/bin/bash

# Ubuntu 20.04 x86_64 のローカルでビルドするスクリプト

cd `dirname $0`

set -ex

# apt install curl git

PACKAGE_NAME=raspberry-pi-os_armv7
PROJECT_DIR=`pwd`/..
SCRIPT_DIR=$PROJECT_DIR/script
BUILDBASE_DIR=$PROJECT_DIR/build/$PACKAGE_NAME
SOURCE_DIR=$PROJECT_DIR/_source/$PACKAGE_NAME
BUILD_DIR=$PROJECT_DIR/_build/$PACKAGE_NAME
INSTALL_DIR=$PROJECT_DIR/_install/$PACKAGE_NAME

mkdir -p $SOURCE_DIR
mkdir -p $BUILD_DIR
mkdir -p $INSTALL_DIR

source $PROJECT_DIR/VERSION
CURRENT_VERSION="`cat $PROJECT_DIR/VERSION`"
INSTALLED_VERSION=""
if [ -e $INSTALL_DIR/VERSION ]; then
  INSTALLED_VERSION="`cat $INSTALL_DIR/VERSION`"
fi

if [ "$CURRENT_VERSION" != "$INSTALLED_VERSION" ]; then
  # RootFS の構築
  $SCRIPT_DIR/init_rootfs_armhf.sh $INSTALL_DIR/rootfs $BUILDBASE_DIR/rpi-raspbian.conf

  # WebRTC の取得
  $SCRIPT_DIR/get_webrtc.sh "$WEBRTC_BUILD_VERSION" raspberry-pi-os_armv7 $INSTALL_DIR $SOURCE_DIR

  # コンパイラの取得
  $SCRIPT_DIR/get_llvm.sh $INSTALL_DIR/webrtc $INSTALL_DIR

  # Boost のビルド
  $SCRIPT_DIR/setup_boost.sh $BOOST_VERSION $SOURCE_DIR/boost $SOURCE_DIR
  pushd $SOURCE_DIR/boost/source
    mkdir -p $BUILD_DIR/boost
    echo "using clang : : $INSTALL_DIR/llvm/clang/bin/clang++ : ;" > project-config.jam
    ./b2 \
      cxxstd=17 \
      cxxflags=" \
        -D_LIBCPP_ABI_UNSTABLE \
        -D_LIBCPP_DISABLE_AVAILABILITY \
        -nostdinc++ \
        -isystem${INSTALL_DIR}/llvm/libcxx/include \
        --target=arm-linux-gnueabihf \
        --sysroot=$INSTALL_DIR/rootfs \
        -I$INSTALL_DIR/rootfs/usr/include/arm-linux-gnueabihf \
      " \
      linkflags=' \
      ' \
      toolset=clang \
      visibility=global \
      target-os=linux \
      architecture=arm \
      address-model=32 \
      link=static \
      variant=release \
      install \
      -j`nproc` \
      --build-dir=$BUILD_DIR/boost \
      --ignore-site-config \
      --prefix=$INSTALL_DIR/boost \
      --with-filesystem \
      --with-json

  # CLI11 の取得
  rm -rf $INSTALL_DIR/CLI11
  git clone --branch v$CLI11_VERSION --depth 1 https://github.com/CLIUtils/CLI11.git $INSTALL_DIR/CLI11

  # CMake のインストール
  $SCRIPT_DIR/get_cmake.sh "$CMAKE_VERSION" linux $INSTALL_DIR
  export PATH="$INSTALL_DIR/cmake/bin:$PATH"

  # SDL2 のビルド
  $SCRIPT_DIR/setup_sdl2.sh "$SDL2_VERSION" $SOURCE_DIR/sdl2
  mkdir -p $BUILD_DIR/sdl2/
  pushd $BUILD_DIR/sdl2
    cmake $SOURCE_DIR/sdl2/source \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=$INSTALL_DIR/SDL2 \
      -DCMAKE_C_COMPILER=$INSTALL_DIR/llvm/clang/bin/clang \
      -DCMAKE_C_COMPILER_TARGET=arm-linux-gnueabihf \
      -DCMAKE_CXX_COMPILER=$INSTALL_DIR/llvm/clang/bin/clang++ \
      -DCMAKE_CXX_COMPILER_TARGET=arm-linux-gnueabihf \
      -DCMAKE_SYSTEM_NAME=Linux \
      -DCMAKE_SYSTEM_PROCESSOR=arm \
      -DCMAKE_FIND_ROOT_PATH=$INSTALL_DIR/rootfs \
      -DCMAKE_FIND_ROOT_PATH_MODE_PROGRAM=NEVER \
      -DCMAKE_FIND_ROOT_PATH_MODE_LIBRARY=BOTH \
      -DCMAKE_FIND_ROOT_PATH_MODE_INCLUDE=BOTH \
      -DCMAKE_FIND_ROOT_PATH_MODE_PACKAGE=BOTH \
      -DCMAKE_SYSROOT=$INSTALL_DIR/rootfs \
      -DSDL_STATIC=ON \
      -DSDL_SHARED=OFF \
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
      -DSDL_DLOPEN=ON \
      -DSDL_SENSOR=ON \
      -DSDL_COCOA=OFF \
      -DSDL_KMSDRM=OFF \
      -DSDL_METAL=OFF \
      -DSDL_OPENGL=ON \
      -DSDL_OPENGLES=ON \
      -DSDL_RPI=OFF \
      -DSDL_VIVANTE=OFF \
      -DSDL_VULKAN=OFF \
      -DSDL_WAYLAND=OFF \
      -DSDL_X11=ON \
      -DSDL_X11_SHARED=OFF \
      -DSDL_X11_XCURSOR=OFF \
      -DSDL_X11_XFIXES=OFF \
      -DSDL_X11_XINERAMA=OFF \
      -DSDL_X11_XINPUT=OFF \
      -DSDL_X11_XRANDR=OFF \
      -DSDL_X11_XSCRNSAVER=OFF \
      -DSDL_X11_XSHAPE=OFF \
      -DSDL_X11_XVM=OFF
    make -j`nproc`
    make install
  popd

  cp $PROJECT_DIR/VERSION $INSTALL_DIR/VERSION
fi

source $INSTALL_DIR/webrtc/VERSIONS

export PATH="$INSTALL_DIR/cmake/bin:$PATH"

pushd $PROJECT_DIR
  MOMO_COMMIT="`git rev-parse HEAD`"
  MOMO_COMMIT_SHORT="`cat $MOMO_COMMIT | cut -b 1-8`"
popd

mkdir -p $BUILD_DIR/momo
pushd $BUILD_DIR/momo
  cmake \
    -DCMAKE_BUILD_TYPE=Release \
    -DMOMO_VERSION=$MOMO_VERSION \
    -DMOMO_COMMIT=$MOMO_COMMIT \
    -DWEBRTC_BUILD_VERSION=$WEBRTC_BUILD_VERSION \
    -DWEBRTC_READABLE_VERSION="$WEBRTC_READABLE_VERSION" \
    -DWEBRTC_COMMIT="$WEBRTC_COMMIT" \
    -DBOOST_ROOT_DIR=$INSTALL_DIR/boost \
    -DCLI11_ROOT_DIR=$INSTALL_DIR/CLI11 \
    -DSDL2_ROOT_DIR=$INSTALL_DIR/SDL2 \
    -DWEBRTC_INCLUDE_DIR=$INSTALL_DIR/webrtc/include \
    -DWEBRTC_LIBRARY_DIR=$INSTALL_DIR/webrtc/lib \
    -DLIBCXX_INCLUDE_DIR=$INSTALL_DIR/llvm/libcxx/include \
    -DTARGET_OS="linux" \
    -DTARGET_OS_LINUX="raspberry-pi-os" \
    -DTARGET_ARCH="arm" \
    -DTARGET_ARCH_ARM="armv7" \
    -DUSE_MMAL_ENCODER=ON \
    -DUSE_H264=ON \
    -DUSE_SDL2=ON \
    -DCLANG_ROOT=$INSTALL_DIR/llvm/clang \
    -DUSE_LIBCXX=ON \
    -DSYSROOT=$INSTALL_DIR/rootfs \
    -DLIBCXX_INCLUDE_DIR=$INSTALL_DIR/llvm/libcxx/include \
    -DBoost_ARCHITECTURE=32 \
    $PROJECT_DIR
  cmake --build . -j`nproc`
popd
