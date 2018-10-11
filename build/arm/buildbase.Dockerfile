# ARM 全体で共通のイメージ
FROM ubuntu:16.04

RUN apt-get update \
   && apt-get -y upgrade \
   && apt-get -y install \
     build-essential \
     curl \
     git \
     gtk+-3.0 \
     lbzip2 \
     libgtk-3-dev \
     lsb-release \
     multistrap \
     python \
     sudo \
     xz-utils \
   && apt-get clean \
   && rm -rf /var/lib/apt/lists/*

RUN set -x \
  && cd /root \
  && git clone https://chromium.googlesource.com/chromium/tools/depot_tools.git ./depot_tools

ENV PATH /root/depot_tools:$PATH
