# syntax = docker/dockerfile:1.1.1-experimental
FROM ubuntu:18.04

ARG PACKAGE_NAME

LABEL jp.shiguredo.momo=$PACKAGE_NAME

RUN rm -f /etc/apt/apt.conf.d/docker-clean; echo 'Binary::apt::APT::Keep-Downloaded-Packages "true";' > /etc/apt/apt.conf.d/keep-cache

# パッケージのインストール

COPY script/apt_install_arm.sh /root/
RUN --mount=type=cache,id=$PACKAGE_NAME,target=/var/cache/apt --mount=type=cache,id=$PACKAGE_NAME,target=/var/lib/apt \
  /root/apt_install_arm.sh

# RootFS の構築

COPY script/init_rootfs_armhf.sh /root/
COPY rpi-raspbian.conf /root/
RUN --mount=type=cache,id=$PACKAGE_NAME,target=/var/cache/apt --mount=type=cache,id=$PACKAGE_NAME,target=/var/lib/apt \
  /root/init_rootfs_armhf.sh /root/rootfs /root/rpi-raspbian.conf

# WebRTC の取得

ARG WEBRTC_BUILD_VERSION

COPY script/get_webrtc.sh /root/
RUN /root/get_webrtc.sh "$WEBRTC_BUILD_VERSION" raspberry-pi-os_armv6 /root /root

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
    cxxstd=17 \
    cxxflags=' \
      -D_LIBCPP_ABI_NAMESPACE=Cr \
      -D_LIBCPP_ABI_VERSION=2 \
      -D_LIBCPP_DISABLE_AVAILABILITY \
      -nostdinc++ \
      -isystem/root/llvm/libcxx/include \
      --target=arm-linux-gnueabihf \
      --sysroot=/root/rootfs \
      -I/root/rootfs/usr/include/arm-linux-gnueabihf \
    ' \
    linkflags=' \
      -L/root/rootfs/usr/lib/arm-linux-gnueabihf \
      -B/root/rootfs/usr/lib/arm-linux-gnueabihf \
    ' \
    toolset=clang \
    visibility=global \
    target-os=linux \
    architecture=arm \
    address-model=32 \
    link=static \
    variant=release \
    install \
    -d+0 \
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
