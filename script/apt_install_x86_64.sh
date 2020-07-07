#!/bin/bash

apt-get update
apt-get -y upgrade

# Ubuntu 18.04 では tzdata を noninteractive にしないと実行が止まってしまう
apt-get -y install tzdata
echo 'Asia/Tokyo' > /etc/timezone
dpkg-reconfigure -f noninteractive tzdata

apt-get -y install \
  build-essential \
  curl \
  git \
  libasound2-dev \
  libc6-dev \
  libexpat1-dev \
  libgtk-3-dev \
  libnspr4-dev \
  libnss3-dev \
  libpulse-dev \
  libudev-dev \
  libxrandr-dev \
  lsb-release \
  python \
  python-dev \
  rsync \
  sudo \
  vim \
  wget
