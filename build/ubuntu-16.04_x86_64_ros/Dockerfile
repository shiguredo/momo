# syntax = docker/dockerfile:1.1.1-experimental
FROM ros:kinetic-ros-core-xenial

ARG PACKAGE_NAME

LABEL jp.shiguredo.momo=$PACKAGE_NAME

RUN rm -f /etc/apt/apt.conf.d/docker-clean; echo 'Binary::apt::APT::Keep-Downloaded-Packages "true";' > /etc/apt/apt.conf.d/keep-cache

COPY script/docker.sh /root/

# 依存パッケージ

RUN --mount=type=cache,id=$PACKAGE_NAME,target=/var/cache/apt --mount=type=cache,id=$PACKAGE_NAME,target=/var/lib/apt \
  /bin/bash -c " \
    set -ex \
    && source /root/docker.sh \
    && apt_install_ubuntu_x86_64 \
    && apt_install_ros_kinetic \
  "

# WebRTC の準備

COPY patch/4k.patch /root/

ARG WEBRTC_VERSION
ARG WEBRTC_COMMIT

RUN --mount=type=cache,id=$PACKAGE_NAME,target=/var/cache/apt --mount=type=cache,id=$PACKAGE_NAME,target=/var/lib/apt --mount=type=cache,id=$PACKAGE_NAME,target=/root/webrtc-cache \
  /bin/bash -c " \
    set -ex \
    && source /root/docker.sh \
    && prepare_webrtc_x86_64 /root/webrtc-cache $WEBRTC_COMMIT /root/webrtc \
    && cd /root/webrtc/src \
    && patch -p2 < /root/4k.patch \
  "

# WebRTC のビルド

ENV PATH /root/webrtc/depot_tools:$PATH

RUN \
  set -ex \
  && cd /root/webrtc/src \
  && gn gen /root/webrtc-build/$PACKAGE_NAME --args='target_os="linux" is_debug=false rtc_include_tests=false rtc_use_h264=false is_component_build=false use_rtti=true use_custom_libcxx=false use_custom_libcxx_for_host=false' \
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
      --prefix=/root/boost-$BOOST_VERSION \
      --with-filesystem \
  "
