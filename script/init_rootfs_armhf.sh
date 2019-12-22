#!/bin/bash

if [ $# -lt 2 ]; then
  echo "$0 <target_dir> <config_file>"
  exit 1
fi

TARGET_DIR=$1
CONFIG_FILE=$2

set -ex

multistrap --no-auth -a armhf -d $TARGET_DIR -f $CONFIG_FILE

find $TARGET_DIR/usr/lib/arm-linux-gnueabihf -lname '/*' -printf '%p %l\n' | while read link target; do ln -snfv "../../..${target}" "${link}"; done
find $TARGET_DIR/usr/lib/arm-linux-gnueabihf/pkgconfig -printf "%f\n" | while read target; do ln -snfv "../../lib/arm-linux-gnueabihf/pkgconfig/${target}" $TARGET_DIR/usr/share/pkgconfig/${target}; done
