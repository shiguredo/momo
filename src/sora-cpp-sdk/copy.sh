#!/bin/bash

# Sora C++ SDK から Momo へファイルをコピーするスクリプト

set -e

cd `dirname $0`

if [ $# -ne 1 ]; then
  echo "Usage: $0 <Sora C++ SDK dir>"
  exit 1
fi

source_dir=$1

find . -mindepth 2 -type f | while read file; do
  if [ ! -f "$source_dir/$file" ]; then
    continue
  fi
  echo "Copying $file from $source_dir"
  cp "$source_dir/$file" "$file"
done
