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

. $WEBRTC_DIR/VERSIONS

# tools の update.py を叩いて特定バージョンの clang バイナリを拾う
mkdir -p $OUTPUT_DIR/llvm
pushd $OUTPUT_DIR/llvm
  if [ ! -e tools/.git ]; then
    git clone https://chromium.googlesource.com/chromium/src/tools
  fi
  pushd tools
    git fetch
    git reset --hard $WEBRTC_SRC_TOOLS_COMMIT
    python clang/scripts/update.py --output-dir=$OUTPUT_DIR/llvm/clang
  popd
popd

# 特定バージョンの libcxx を利用する
pushd $OUTPUT_DIR/llvm
  if [ ! -e libcxx/.git ]; then
    git clone https://chromium.googlesource.com/external/github.com/llvm/llvm-project/libcxx
  fi
  pushd libcxx
    git fetch
    git reset --hard $WEBRTC_SRC_BUILDTOOLS_THIRD_PARTY_LIBCXX_TRUNK
  popd
popd
