#!/bin/bash

# src/ 以下の全てのファイルに clang-format を適用する

set -e

for file in `find src -type f`; do
  echo applying $file
  clang-format -i -style=file $file
done
