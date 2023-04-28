#!/bin/bash

set -ex

SYSDIR=/root/rootfs

pushd $SYSDIR/usr/lib/aarch64-linux-gnu/tegra/
  ln -sf libnvbuf_utils.so.1.0.0 libnvbuf_utils.so
  ln -s libnvbuf_fdmap.so.1.0.0 libnvbuf_fdmap.so
popd
