#!/bin/bash

# <platform> には Linux か Darwin を指定する
# <output_dir>/cmake に CMake が配置される
if [ $# -lt 3 ]; then
  echo "$0 <cmake_version> <platform> <output_dir>"
  exit 1
fi

CMAKE_VERSION=$1
PLATFORM=$2
OUTPUT_DIR=$3

set -ex

pushd $OUTPUT_DIR
  curl -LO https://github.com/Kitware/CMake/releases/download/v${CMAKE_VERSION}/cmake-${CMAKE_VERSION}-${PLATFORM}-x86_64.tar.gz
  tar xf cmake-${CMAKE_VERSION}-${PLATFORM}-x86_64.tar.gz
  rm cmake-${CMAKE_VERSION}-${PLATFORM}-x86_64.tar.gz
  rm -rf cmake
  mv cmake-${CMAKE_VERSION}-${PLATFORM}-x86_64 cmake
popd
