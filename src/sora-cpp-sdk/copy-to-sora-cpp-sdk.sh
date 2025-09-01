#!/bin/bash

# Momo から Sora C++ SDK へファイルをコピーするスクリプト

set -e

cd `dirname $0`

if [ $# -ne 1 ]; then
  echo "Usage: $0 <Sora C++ SDK dir>"
  exit 1
fi

source_dir=$1

INCLUDE_FILES=$(find include -type f)
SRC_FILES=$(find src -type f)

ALL_FILES=$(echo -e "$INCLUDE_FILES\n$SRC_FILES" | sort)

echo "$ALL_FILES" | while read file; do
  if [ ! -d $(dirname "$source_dir/$file") ]; then
    continue
  fi
  echo "Copying $file from $source_dir"
  cp "$file" "$source_dir/$file"
done
