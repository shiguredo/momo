#!/bin/bash

if [ $# -lt 2 ]; then
  echo "$0 <target_dir> <config_file>"
  exit 1
fi

TARGET_DIR=$1
CONFIG_FILE=$2

set -ex

multistrap --no-auth -a arm64 -d $TARGET_DIR -f $CONFIG_FILE
find $TARGET_DIR/usr/lib/aarch64-linux-gnu -lname '/*' -printf '%p %l\n' | while read link target; do ln -snfv "../../..${target}" "${link}"; done
find $TARGET_DIR/usr/lib/aarch64-linux-gnu/pkgconfig -printf "%f\n" | while read target; do ln -snfv "../../lib/aarch64-linux-gnu/pkgconfig/${target}" $TARGET_DIR/usr/share/pkgconfig/${target}; done
if [ -L $TARGET_DIR/usr/lib/gcc/aarch64-linux-gnu/5/libgcc_s.so ]; then
  unlink $TARGET_DIR/usr/lib/gcc/aarch64-linux-gnu/5/libgcc_s.so
  ln -s ../../../../../lib64/aarch64-linux-gnu/libgcc_s.so.1 $TARGET_DIR/usr/lib/gcc/aarch64-linux-gnu/5/libgcc_s.so
elif [ -L $TARGET_DIR/usr/lib/gcc/aarch64-linux-gnu/7/libgcc_s.so ]; then
  unlink $TARGET_DIR/usr/lib/gcc/aarch64-linux-gnu/7/libgcc_s.so
  ln -s ../../../../../lib64/aarch64-linux-gnu/libgcc_s.so.1 $TARGET_DIR/usr/lib/gcc/aarch64-linux-gnu/7/libgcc_s.so
elif [ -L $TARGET_DIR/usr/lib/gcc/aarch64-linux-gnu/8/libgcc_s.so ]; then
  unlink $TARGET_DIR/usr/lib/gcc/aarch64-linux-gnu/8/libgcc_s.so
  if [ -e $TARGET_DIR/lib64/aarch64-linux-gnu/libgcc_s.so.1 ]; then
    ln -s ../../../../../lib64/aarch64-linux-gnu/libgcc_s.so.1 $TARGET_DIR/usr/lib/gcc/aarch64-linux-gnu/8/libgcc_s.so
  elif [ -e $TARGET_DIR/lib/aarch64-linux-gnu/libgcc_s.so.1 ]; then
    ln -s ../../../../../lib/aarch64-linux-gnu/libgcc_s.so.1 $TARGET_DIR/usr/lib/gcc/aarch64-linux-gnu/8/libgcc_s.so
  else
    exit 1
  fi
else
  exit 1
fi
