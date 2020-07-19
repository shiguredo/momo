#!/bin/bash

set -ex

SYSDIR=/root/rootfs

pushd $SYSDIR/usr/lib/aarch64-linux-gnu
  # 既存の libdl.so は libdl.so -> ../../../lib/aarch64-linux-gnu/libdl.so.2 なのに対して、
  # Jetson Nano の libdl.so は libdl.so -> /lib/aarch64-linux-gnu/libdl.so.2 になっているため、パスが見つからない。
  # なので symlink を相対パスで貼り直してやる。
  ln -sf ../../../lib/aarch64-linux-gnu/libdl.so.2 libdl.so
  pushd tegra
    # libnvbuf_utils.so.1.0.0 も同じ
    ln -sf libnvbuf_utils.so.1.0.0 libnvbuf_utils.so
  popd
popd

pushd $SYSDIR/usr/lib/aarch64-linux-gnu/tegra/
  ln -s libnvbuf_fdmap.so.1.0.0 libnvbuf_fdmap.so
popd
