#!/bin/bash

if [ $# -eq 0 ]; then
  echo "説明: バージョン情報の変数を設定した上で指定した引数の処理を呼ぶスクリプト"
  echo ""
  echo "使い方: $0 [任意の引数]..."
  exit 0
fi

set -e

SCRIPT_DIR=`cd $(dirname $0); pwd`

# バージョン情報の取得
source $SCRIPT_DIR/../VERSION

# export する
export MOMO_VERSION
export WEBRTC_BRANCH
export WEBRTC_COMMIT
export BOOST_VERSION
export JSON_VERSION
export CLI11_VERSION

# Makefile に渡す用
VERSIONS_MAKEFILE="
  MOMO_VERSION=$MOMO_VERSION \
  WEBRTC_BRANCH=$WEBRTC_BRANCH \
  WEBRTC_COMMIT=$WEBRTC_COMMIT \
  BOOST_VERSION=$BOOST_VERSION \
  JSON_VERSION=$JSON_VERSION \
  CLI11_VERSION=$CLI11_VERSION \
"

# Docker に渡す用
VERSIONS_DOCKER="
  --build-arg MOMO_VERSION=$MOMO_VERSION \
  --build-arg WEBRTC_BRANCH=$WEBRTC_BRANCH \
  --build-arg WEBRTC_COMMIT=$WEBRTC_COMMIT \
  --build-arg BOOST_VERSION=$BOOST_VERSION \
  --build-arg JSON_VERSION=$JSON_VERSION \
  --build-arg CLI11_VERSION=$CLI11_VERSION \
"

# C/C++ に渡す用
VERSIONS_C_CPP="
  -DMOMO_VERSION=$MOMO_VERSION \
  -DWEBRTC_BRANCH=$WEBRTC_BRANCH \
  -DWEBRTC_COMMIT=$WEBRTC_COMMIT \
  -DBOOST_VERSION=$BOOST_VERSION \
  -DJSON_VERSION=$JSON_VERSION \
  -DCLI11_VERSION=$CLI11_VERSION \
"

exec "$@"
