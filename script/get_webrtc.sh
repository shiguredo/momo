#!/bin/bash

# <output_dir>/webrtc に WebRTC ライブラリが配置される
if [ $# -lt 3 ]; then
  echo "$0 <webrtc_build_version> <package_name> <output_dir>"
  exit 1
fi

WEBRTC_BUILD_VERSION=$1
PACKAGE_NAME=$2
OUTPUT_DIR=$3

set -ex

pushd $OUTPUT_DIR
  curl -LO https://github.com/melpon/webrtc-build/releases/download/m${WEBRTC_BUILD_VERSION}/webrtc.${PACKAGE_NAME}.tar.gz
  tar xf webrtc.${PACKAGE_NAME}.tar.gz
  rm webrtc.${PACKAGE_NAME}.tar.gz
popd
