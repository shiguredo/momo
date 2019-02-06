#!/bin/bash

# 各 Dockerfile から呼び出す共通の処理を記述したスクリプト。
# source で読み込んで利用する。
#
# 先頭が _ で始まる関数は private 扱いの関数という意味なので、外から呼んではいけない。

# ARM 用の共通パッケージをインストールする
#
# 引数: なし
function _apt_install_arm_packages() {
  apt-get update || return 1
  apt-get -y upgrade || return 1
  apt-get -y install \
    build-essential \
    curl \
    git \
    gtk+-3.0 \
    lbzip2 \
    libgtk-3-dev \
    lsb-release \
    multistrap \
    python \
    sudo \
    vim \
    xz-utils || return 1
}

# 32bit ARM 用の rootfs を構築する
#
# 引数:
#   - $1: rootfs の構築ディレクトリ
#   - $2: 設定ファイルのパス
function init_rootfs_armhf() {
  local target_dir=$1
  local config_file=$2
  _apt_install_arm_packages || return 1
  multistrap --no-auth -a armhf -d $target_dir -f $config_file || return 1
  (find $target_dir/usr/lib/arm-linux-gnueabihf -lname '/*' -printf '%p %l\n' | while read link target; do ln -snfv "../../..${target}" "${link}"; done) || return 1
  (find $target_dir/usr/lib/arm-linux-gnueabihf/pkgconfig -printf "%f\n" | while read target; do ln -snfv "../../lib/arm-linux-gnueabihf/pkgconfig/${target}" $target_dir/usr/share/pkgconfig/${target}; done) || return 1
}

# 64bit ARM 用の rootfs を構築する
#
# 引数:
#   - $1: rootfs の構築ディレクトリ
#   - $2: 設定ファイルのパス
function init_rootfs_arm64() {
  local target_dir=$1
  local config_file=$2
  _apt_install_arm_packages || return 1
  multistrap --no-auth -a arm64 -d $target_dir -f $config_file || return 1
  (find $target_dir/usr/lib/aarch64-linux-gnu -lname '/*' -printf '%p %l\n' | while read link target; do ln -snfv "../../..${target}" "${link}"; done) || return 1
  (find $target_dir/usr/lib/aarch64-linux-gnu/pkgconfig -printf "%f\n" | while read target; do ln -snfv "../../lib/aarch64-linux-gnu/pkgconfig/${target}" $target_dir/usr/share/pkgconfig/${target}; done) || return 1
  unlink $target_dir/usr/lib/gcc/aarch64-linux-gnu/5/libgcc_s.so || return 1
  ln -s ../../../../../lib64/aarch64-linux-gnu/libgcc_s.so.1 $target_dir/usr/lib/gcc/aarch64-linux-gnu/5/libgcc_s.so || return 1
}

# Ubuntu+x86_64 用の共通パッケージをインストールする
#
# 引数: なし
function apt_install_ubuntu_x86_64() {
  apt-get update
  apt-get -y upgrade
  apt-get -y install \
    git \
    lsb-release \
    python \
    sudo \
    vim \
    wget
}

# git clone するけど、既存のディレクトリがある場合はうまいこと既存のを利用する
#
# 引数:
#   - $1: リモートのURL
#   - $2: clone 先のディレクトリ
function _git_clone_awesome() {
  local url=$1
  local target_dir=$2
  if [ -e $target_dir ]; then
    # 既にディレクトリが存在しているので、うまいことリセットする
    cd $target_dir || return 1
    git reset --hard || return 1
    git fetch || return 1
    git checkout -f origin/HEAD || return 1
    git clean -xdf || return 1
  else
    git clone $url $target_dir || return 1
    cd $target_dir || return 1
    git checkout -f origin/HEAD || return 1
  fi
}

# WebRTC をビルドするための準備をする
#
# - depot_tools の clone
# - apt を使って必要なライブラリのインストール(Linux の場合のみ)
# - 指定したバージョンのソースをダウンロード（あるいはアップデート）
#
# までを行う。適用したパッチは消える。
# あとは gn gen して ninja すれば libwebrtc.a が生成できる。
#
# また、WebRTC のソースをキャッシュディレクトリに保存した後に構築ディレクトリにコピーする。
# そのためキャッシュディレクトリの内容をキャッシュしても構わない。
#
# 引数:
#   $1: 対象が arm か x86_64 か macos か
#   $2: 一時ファイル置き場ディレクトリ。$1/depot_tools や $1/src などのディレクトリが生成される。
#       空文字列だった場合は $4 に直接配置される。
#   $3: WebRTC のコミットハッシュ
#   $4: 構築ディレクトリ。キャッシュディレクトリのコピーが配置される
function _prepare_webrtc() {
  local target_arch=$1
  local cache_dir=$2
  local webrtc_commit=$3
  local target_dir=$4

  # キャッシュディレクトリが指定されていない場合、
  # 構築ディレクトリを利用する。
  if [ -z "$cache_dir" ]; then
    local cache_dir="$target_dir"
    local use_cache_dir=0
  else
    local use_cache_dir=1
  fi

  _git_clone_awesome https://chromium.googlesource.com/chromium/tools/depot_tools.git $cache_dir/depot_tools || return 1
  export PATH=$cache_dir/depot_tools:$PATH

  cd $cache_dir || return 1
  if [ ! -e src ]; then
    fetch webrtc || return 1
  fi
  if [ "$target_arch" = "arm" ]; then
    apt-get update || return 1
    bash ./src/build/install-build-deps.sh || return 1
  elif [ "$target_arch" = "x86_64" ]; then
    sed -i -e 's/sudo/sudo -E/g' ./src/build/install-build-deps.sh || return 1
    bash ./src/build/install-build-deps.sh --no-arm --no-chromeos-fonts || return 1
    (pushd src/build && git reset --hard && popd) || return 1
  elif [ "$target_arch" = "macos" ]; then
    :
  fi
  gclient sync || return 1

  cd $cache_dir/src || return 1
  git checkout -f $webrtc_commit || return 1
  gclient sync || return 1

  # キャッシュディレクトリと構築ディレクトリが別だった場合、
  # キャッシュディレクトリの内容を構築ディレクトリへコピーする
  if [ $use_cache_dir -eq 1 ]; then
    cp -r $cache_dir $target_dir || return 1
  fi
}

# _prepare_webrtc を参照
function prepare_webrtc_arm() {
  _prepare_webrtc arm "$@"
}

# _prepare_webrtc を参照
function prepare_webrtc_x86_64() {
  _prepare_webrtc x86_64 "$@"
}

# _prepare_webrtc を参照
function prepare_webrtc_macos() {
  _prepare_webrtc macos "$@"
}

# 指定した Boost のバージョンをダウンロードして、b2 を実行できる状態にする
#
# ソースコードは $2/source に配置されるので、ここに cd した上で ./b2 を実行してビルドすること
#
# 引数:
#   $1: Boost のバージョン
#   $2: 出力ディレクトリ
function setup_boost() {
  local boost_version=$1
  local boost_version_underscore=${boost_version//./_}
  local output_dir=$2

  mkdir -p $output_dir
  cd $output_dir || return 1
  if [ ! -e boost_$boost_version_underscore.tar.gz ]; then
    curl -LO https://dl.bintray.com/boostorg/release/$boost_version/source/boost_$boost_version_underscore.tar.gz || return 1
  fi
  if [ ! -e source ]; then
    tar xf boost_$boost_version_underscore.tar.gz || return 1
    mv boost_$boost_version_underscore source || return 1
  fi

  cd source || return 1
  if [ ! -e b2 ]; then
    ./bootstrap.sh || return 1
  fi
}

