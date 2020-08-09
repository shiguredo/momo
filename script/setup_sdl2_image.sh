#!/bin/bash

# 指定した SDL2_image のバージョンをダウンロードして、make を実行できる状態にする
#
# ソースコードは $2/source に配置される
#
# 引数:
#   $1: SDL2_image のバージョン
#   $2: 出力ディレクトリ
if [ $# -lt 2 ]; then
  echo "$0 <sdl2_image_version> <output_dir>"
  exit 1
fi

SDL2_IMAGE_VERSION=$1
OUTPUT_DIR=$2

mkdir -p $OUTPUT_DIR
pushd $OUTPUT_DIR
  if [ ! -e SDL2_image-$SDL2_IMAGE_VERSION.tar.gz ]; then
    curl -LO https://www.libsdl.org/projects/SDL_image/release/SDL2_image-$SDL2_IMAGE_VERSION.tar.gz
  fi
  rm -rf source
  rm -rf SDL2_image-$SDL2_IMAGE_VERSION
  tar xf SDL2_image-$SDL2_IMAGE_VERSION.tar.gz
  mv SDL2_image-$SDL2_IMAGE_VERSION source
popd