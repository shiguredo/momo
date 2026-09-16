#!/bin/bash

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

if [ $# -ne 2 ]; then
  echo "使用方法: $0 <Sora C++ SDK dir> <target_commit>"
  echo "例: $0 ../../../sora-cpp-sdk HEAD"
  echo "例: $0 ../../../sora-cpp-sdk 2025.5.0"
  echo "例: $0 ../../../sora-cpp-sdk abc123def"
  exit 1
fi

SORA_CPP_SDK_DIR="`pwd`/$1"
if [ ! -d $SORA_CPP_SDK_DIR ]; then
  echo "sora-cpp-sdk リポジトリが存在しません"
  exit 1
fi

cd "$SCRIPT_DIR"

TARGET_REF="$2"

source LAST_UPDATED

INCLUDE_FILES=$(find include -type f)
SRC_FILES=$(find src -type f)
THIRD_PARTY_FILES=$(find third_party -type f)

ALL_FILES=$(echo -e "$INCLUDE_FILES\n$SRC_FILES\n$THIRD_PARTY_FILES" | sort)

pushd $SORA_CPP_SDK_DIR
  git fetch --all --tags

  BASE_COMMIT=$(git rev-parse "$LAST_UPDATED_SORA_CPP_SDK")
  TARGET_COMMIT=$(git rev-parse "$TARGET_REF")

  echo "==================== 差分 ===================="
  echo "FROM: $LAST_UPDATED_SORA_CPP_SDK ($BASE_COMMIT)"
  echo "TO:   $TARGET_REF ($TARGET_COMMIT)"
  echo ""

  git --no-pager diff "$LAST_UPDATED_SORA_CPP_SDK..$TARGET_REF" -- $ALL_FILES

  echo ""
  echo "==================== サマリー ===================="
  echo "コミット範囲: $LAST_UPDATED_SORA_CPP_SDK..$TARGET_REF"
  echo ""
  echo "コミットログ:"
  git --no-pager log --oneline "$LAST_UPDATED_SORA_CPP_SDK..$TARGET_REF" -- $ALL_FILES
popd
