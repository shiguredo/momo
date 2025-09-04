#!/bin/bash

# Sora C++ SDK と Momo の同期が完了した後に、Momo 側の LAST_UPDATED を更新するスクリプト

set -e

cd `dirname $0`

if [ $# -ne 1 ]; then
  echo "Usage: $0 <Sora C++ SDK dir>"
  exit 1
fi

SORA_CPP_SDK_DIR=$1

if [ ! -d $SORA_CPP_SDK_DIR ]; then
  echo "sora-cpp-sdk リポジトリが存在しません"
  exit 1
fi

SORA_CPP_SDK_COMMIT=$(git -C $SORA_CPP_SDK_DIR rev-parse HEAD)
MOMO_COMMIT=$(git rev-parse HEAD)

echo "LAST_UPDATED_MOMO=$MOMO_COMMIT" > LAST_UPDATED
echo "LAST_UPDATED_SORA_CPP_SDK=$SORA_CPP_SDK_COMMIT" >> LAST_UPDATED
