# syntax = docker/dockerfile:1.1.1-experimental
FROM ubuntu:20.04

ARG PACKAGE_NAME

LABEL jp.shiguredo.momo=$PACKAGE_NAME

RUN rm -f /etc/apt/apt.conf.d/docker-clean; echo 'Binary::apt::APT::Keep-Downloaded-Packages "true";' > /etc/apt/apt.conf.d/keep-cache

# パッケージのインストール

COPY script/apt_install_x86_64.sh /root/
RUN --mount=type=cache,id=$PACKAGE_NAME,target=/var/cache/apt --mount=type=cache,id=$PACKAGE_NAME,target=/var/lib/apt \
  /root/apt_install_x86_64.sh

# WebRTC の取得

ARG WEBRTC_BUILD_VERSION

COPY script/get_webrtc.sh /root/
RUN /root/get_webrtc.sh "$WEBRTC_BUILD_VERSION" ubuntu-20.04_x86_64 /root /root
# COPY webrtc/ /root/webrtc/

# コンパイラの取得

COPY script/get_llvm.sh /root/
RUN /root/get_llvm.sh /root/webrtc /root

# Boost のビルド

ARG BOOST_VERSION

COPY _cache/boost/ /root/_cache/boost/
COPY script/setup_boost.sh /root/
RUN \
  set -ex \
  && /root/setup_boost.sh "$BOOST_VERSION" /root/boost-source /root/_cache/boost \
  && cd /root/boost-source/source \
  && echo 'using clang : : /root/llvm/clang/bin/clang++ : ;' > project-config.jam \
  && ./b2 \
    cxxflags=' \
      -D_LIBCPP_ABI_UNSTABLE \
      -nostdinc++ \
      -isystem/root/llvm/libcxx/include \
    ' \
    linkflags=' \
    ' \
    toolset=clang \
    visibility=global \
    target-os=linux \
    address-model=64 \
    link=static \
    variant=release \
    install \
    -j`nproc` \
    --ignore-site-config \
    --prefix=/root/boost \
    --with-filesystem \
    --with-json

# CLI11 の取得

ARG CLI11_VERSION
RUN git clone --branch v$CLI11_VERSION --depth 1 https://github.com/CLIUtils/CLI11.git /root/CLI11

# CMake のインストール
ARG CMAKE_VERSION
COPY script/get_cmake.sh /root/
RUN /root/get_cmake.sh "$CMAKE_VERSION" linux /root
ENV PATH "/root/cmake/bin:$PATH"

# SDL2 のビルド

ARG SDL2_VERSION

COPY script/setup_sdl2.sh /root/
RUN \
  set -ex \
  && /root/setup_sdl2.sh "$SDL2_VERSION" /root/sdl2-source \
  && mkdir -p /root/sdl2-source/build \
  && cd /root/sdl2-source/build \
  && cmake ../source \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/root/SDL2 \
    -DCMAKE_C_COMPILER=/root/llvm/clang/bin/clang \
    -DCMAKE_CXX_COMPILER=/root/llvm/clang/bin/clang++ \
    -DBUILD_SHARED_LIBS=OFF \
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
    -DVIDEO_OPENGL=ON \
    -DVIDEO_OPENGLES=ON \
    -DVIDEO_RPI=OFF \
    -DVIDEO_WAYLAND=OFF \
    -DVIDEO_X11=ON \
    -DX11_SHARED=OFF \
    -DVIDEO_X11_XCURSOR=OFF \
    -DVIDEO_X11_XINERAMA=OFF \
    -DVIDEO_X11_XINPUT=OFF \
    -DVIDEO_X11_XRANDR=OFF \
    -DVIDEO_X11_XSCRNSAVER=OFF \
    -DVIDEO_X11_XSHAPE=OFF \
    -DVIDEO_X11_XVM=OFF \
    -DVIDEO_VULKAN=OFF \
    -DVIDEO_VIVANTE=OFF \
    -DVIDEO_COCOA=OFF \
    -DVIDEO_METAL=OFF \
    -DVIDEO_KMSDRM=OFF \
  && make -j`nproc` \
  && make install

# CUDA 周りのインストール
ARG CUDA_VERSION
RUN set -ex \
  && apt-get update \
  && apt-get install -y software-properties-common \
  && wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/cuda-ubuntu2004.pin \
  && mv cuda-ubuntu2004.pin /etc/apt/preferences.d/cuda-repository-pin-600 \
  && apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/7fa2af80.pub \
  && add-apt-repository "deb http://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/ /" \
  && apt-get update \
  && DEBIAN_FRONTEND=noninteractive apt-get -y install cuda=$CUDA_VERSION clang-10
