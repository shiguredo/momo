#!/bin/bash

# src/ 以下の全てのファイルに clang-format を適用する

set -e

for file in `find src -type f \( -name '*.h' -o -name '*.cpp' -o -name '*.mm' \)`; do
  echo applying $file
  clang-format -i -style=file $file
done
