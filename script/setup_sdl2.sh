#!/bin/bash

# 指定した SDL2 のバージョンをダウンロードして、make を実行できる状態にする
#
# ソースコードは $2/source に配置される
#
# 引数:
#   $1: SDL2 のバージョン
#   $2: 出力ディレクトリ
if [ $# -lt 2 ]; then
  echo "$0 <sdl2_version> <output_dir>"
  exit 1
fi

SDL2_VERSION=$1
OUTPUT_DIR=$2

mkdir -p $OUTPUT_DIR
pushd $OUTPUT_DIR
  if [ ! -e SDL2-$SDL2_VERSION.tar.gz ]; then
    curl -LO http://www.libsdl.org/release/SDL2-$SDL2_VERSION.tar.gz
  fi
  rm -rf source
  rm -rf SDL2-$SDL2_VERSION
  tar xf SDL2-$SDL2_VERSION.tar.gz
  mv SDL2-$SDL2_VERSION source
popd
