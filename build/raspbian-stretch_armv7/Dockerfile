# syntax = docker/dockerfile:1.1.1-experimental
FROM ubuntu:16.04

ARG PACKAGE_NAME

LABEL jp.shiguredo.momo=$PACKAGE_NAME

RUN rm -f /etc/apt/apt.conf.d/docker-clean; echo 'Binary::apt::APT::Keep-Downloaded-Packages "true";' > /etc/apt/apt.conf.d/keep-cache

COPY script/docker.sh /root/

# RootFS の構築

COPY patch/rpi-raspbian.conf /root/

RUN --mount=type=cache,id=$PACKAGE_NAME,target=/var/cache/apt --mount=type=cache,id=$PACKAGE_NAME,target=/var/lib/apt \
  /bin/bash -c " \
    set -ex \
    && source /root/docker.sh \
    && init_rootfs_armhf /root/rootfs /root/rpi-raspbian.conf \
  "

COPY patch/4k.patch /root/

# WebRTC の準備

ARG WEBRTC_VERSION
ARG WEBRTC_COMMIT

RUN --mount=type=cache,id=$PACKAGE_NAME,target=/var/cache/apt --mount=type=cache,id=$PACKAGE_NAME,target=/var/lib/apt --mount=type=cache,id=$PACKAGE_NAME,target=/root/webrtc-cache \
  /bin/bash -c " \
    set -ex \
    && source /root/docker.sh \
    && prepare_webrtc_arm /root/webrtc-cache $WEBRTC_COMMIT /root/webrtc \
    && cd /root/webrtc/src \
    && patch -p2 < /root/4k.patch \
  "

# WebRTC のビルド

ENV PATH /root/webrtc/depot_tools:$PATH

RUN \
  set -ex \
  && cd /root/webrtc/src \
  && gn gen /root/webrtc-build/$PACKAGE_NAME --args='target_os="linux" target_cpu="arm" is_debug=false target_sysroot="/root/rootfs" rtc_use_h264=true rtc_include_tests=false is_desktop_linux=false rtc_build_json=true use_rtti=true' \
  && ninja -C /root/webrtc-build/$PACKAGE_NAME

# Boost のビルド

ARG BOOST_VERSION

RUN \
  /bin/bash -c " \
    set -ex \
    && source /root/docker.sh \
    && setup_boost $BOOST_VERSION /root/boost-source \
    && cd /root/boost-source/source \
    && echo 'using clang : : /root/webrtc/src/third_party/llvm-build/Release+Asserts/bin/clang++ : ;' > project-config.jam \
    && ./b2 \
      cxxflags=' \
        -D_LIBCPP_ABI_UNSTABLE \
        -nostdinc++ \
        -isystem/root/webrtc/src/buildtools/third_party/libc++/trunk/include \
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
      -j`nproc` \
      --ignore-site-config \
      --prefix=/root/boost-$BOOST_VERSION \
      --with-filesystem \
  "
