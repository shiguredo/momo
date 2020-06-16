#!/bin/bash

set -ex

SYSDIR=/root/rootfs
WORK_DIR=/root/jetson

mkdir -p $WORK_DIR
cd $WORK_DIR

L4T_DRIVER_PACKAGE=Tegra210_Linux_R32.4.2_aarch64.tbz2
L4T_DRIVER_PACKAGE_URL=https://developer.nvidia.com/embedded/L4T/r32_Release_v4.2/t210ref_release_aarch64/$L4T_DRIVER_PACKAGE
MULTIMEDIA_API=Tegra_Multimedia_API_R32.2.3_aarch64.tbz2
MULTIMEDIA_API_URL=https://developer.nvidia.com/embedded/r32-2-3_Release_v1.0/t210ref_release_aarch64/$MULTIMEDIA_API
SAMPLE_ROOT_FILESYSTEM=Tegra_Linux_Sample-Root-Filesystem_R32.4.2_aarch64.tbz2
SAMPLE_ROOT_FILESYSTEM_URL=https://developer.nvidia.com/embedded/L4T/r32_Release_v4.2/t210ref_release_aarch64/$SAMPLE_ROOT_FILESYSTEM

# https://developer.nvidia.com/embedded/linux-tegra から必要なファイルをダウンロードしてくる

if [ ! -e $L4T_DRIVER_PACKAGE ]; then
  curl -LO $L4T_DRIVER_PACKAGE_URL
fi
if [ ! -e $MULTIMEDIA_API ]; then
  curl -LO $MULTIMEDIA_API_URL
fi
if [ ! -e $SAMPLE_ROOT_FILESYSTEM ]; then
  curl -LO $SAMPLE_ROOT_FILESYSTEM_URL
fi

# Tegra Multimedia API
# 必要なファイルだけ展開する
pushd $SYSDIR
  tar xvf $WORK_DIR/$MULTIMEDIA_API --strip-components=1 tegra_multimedia_api/include/
popd

mkdir -p $SYSDIR/usr/src/nvidia
pushd $SYSDIR/usr/src/nvidia
  # ほんとは全部展開するのが正しいけど、必要なのはソースだけなのでそこだけ展開する
  tar xvf $WORK_DIR/$MULTIMEDIA_API tegra_multimedia_api/samples/common/classes
popd

# nvidia drivers
# 何も考えずに全部展開する
tar xvf $L4T_DRIVER_PACKAGE Linux_for_Tegra/nv_tegra/nvidia_drivers.tbz2
pushd $SYSDIR
  tar xvf $WORK_DIR/Linux_for_Tegra/nv_tegra/nvidia_drivers.tbz2
popd

# filesystem
pushd $SYSDIR
  # ライブラリ系とインクルード系だけ展開する
  tar \
    -xvf $WORK_DIR/$SAMPLE_ROOT_FILESYSTEM \
    --wildcards \
    lib/aarch64-linux-gnu \
    'usr/lib/aarch64-linux-gnu/*.so*' \
    usr/include \
    --exclude 'lib/aarch64-linux-gnu/*/*'
popd

pushd $SYSDIR/usr/lib/aarch64-linux-gnu
  # 既存の libdl.so は libdl.so -> ../../../lib/aarch64-linux-gnu/libdl.so.2 なのに対して、
  # Jetson Nano の libdl.so は libdl.so -> /lib/aarch64-linux-gnu/libdl.so.2 になっているため、パスが見つからない。
  # なので symlink を相対パスで貼り直してやる。
  ln -sf ../../../lib/aarch64-linux-gnu/libdl.so.2 libdl.so
  # libz.so も同じ。
  ln -sf ../../../lib/aarch64-linux-gnu/libz.so.1.2.11 libz.so
  pushd tegra
    # libnvbuf_utils.so.1.0.0 も同じ
    ln -sf libnvbuf_utils.so.1.0.0 libnvbuf_utils.so
  popd
popd

# symlink
pushd $SYSDIR/usr/lib/aarch64-linux-gnu/
   ln -s libv4l2.so.0.0.999999 libv4l2.so
popd
pushd $SYSDIR/usr/lib/aarch64-linux-gnu/tegra/
  ln -s libnvbuf_fdmap.so.1.0.0 libnvbuf_fdmap.so
popd

# cleanup
rm -rf $WORK_DIR
