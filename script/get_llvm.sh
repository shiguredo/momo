#!/bin/bash

# WebRTC のビルドに使っていたのと同じバージョンの clang と libcxx を拾ってくる。
#
# 以下の構成になる。
# <output_dir>/llvm/clang
# <output_dir>/llvm/libcxx
# <output_dir>/llvm/tools
if [ $# -lt 2 ]; then
  echo "$0 <webrtc_dir> <output_dir>"
  exit 1
fi

WEBRTC_DIR=$1
OUTPUT_DIR=$2

set -ex

if [ -e $WEBRTC_DIR/VERSIONS ]; then
  . $WEBRTC_DIR/VERSIONS
else
  . $WEBRTC_DIR/release/VERSIONS
fi

# tools の update.py を叩いて特定バージョンの clang バイナリを拾う
mkdir -p $OUTPUT_DIR/llvm
pushd $OUTPUT_DIR/llvm
  if [ ! -e tools/.git ]; then
    git clone $WEBRTC_SRC_TOOLS_URL
  fi
  pushd tools
    git fetch
    git reset --hard $WEBRTC_SRC_TOOLS_COMMIT
    python3 clang/scripts/update.py --output-dir=$OUTPUT_DIR/llvm/clang
  popd
popd

# 特定バージョンの libcxx を利用する
pushd $OUTPUT_DIR/llvm
  if [ ! -e libcxx/.git ]; then
    git clone $WEBRTC_SRC_BUILDTOOLS_THIRD_PARTY_LIBCXX_TRUNK_URL
  fi
  pushd libcxx
    git fetch
    git reset --hard $WEBRTC_SRC_BUILDTOOLS_THIRD_PARTY_LIBCXX_TRUNK_COMMIT
  popd
popd

# __config_site のために特定バージョンの buildtools を取得する
pushd $OUTPUT_DIR/llvm
  if [ ! -e buildtools/.git ]; then
    git clone $WEBRTC_SRC_BUILDTOOLS_URL
  fi
  pushd buildtools
    git fetch
    git reset --hard $WEBRTC_SRC_BUILDTOOLS_COMMIT
  popd
  cp $OUTPUT_DIR/llvm/buildtools/third_party/libc++/__config_site $OUTPUT_DIR/llvm/libcxx/include/
popd
