#!/bin/bash

# <output_dir>/webrtc に WebRTC ライブラリが配置される
# ダウンロードしたファイルは <source_dir>/ に配置される
if [ $# -lt 4 ]; then
  echo "$0 <webrtc_build_version> <package_name> <output_dir> <source_dir>"
  exit 1
fi

WEBRTC_BUILD_VERSION=$1
PACKAGE_NAME=$2
OUTPUT_DIR=$3
SOURCE_DIR=$4

set -ex

if [ ! -e $SOURCE_DIR/webrtc.${PACKAGE_NAME}.${WEBRTC_BUILD_VERSION}.tar.gz ]; then
  curl -Lo $SOURCE_DIR/webrtc.${PACKAGE_NAME}.${WEBRTC_BUILD_VERSION}.tar.gz https://github.com/shiguredo-webrtc-build/webrtc-build/releases/download/m${WEBRTC_BUILD_VERSION}/webrtc.${PACKAGE_NAME}.tar.gz
fi

pushd $OUTPUT_DIR
  tar xf $SOURCE_DIR/webrtc.${PACKAGE_NAME}.${WEBRTC_BUILD_VERSION}.tar.gz
popd
