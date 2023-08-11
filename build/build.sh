#!/bin/bash

cd "`dirname $0`"

# 引数の処理

PROGRAM="$0"

_PACKAGES=" \
  windows \
  macos_arm64 \
  raspberry-pi-os_armv6 \
  raspberry-pi-os_armv7 \
  raspberry-pi-os_armv8 \
  ubuntu-20.04_armv8_jetson_xavier \
  ubuntu-20.04_x86_64 \
  ubuntu-22.04_x86_64 \
"

function show_help() {
  echo "$PROGRAM [--clean] [--package] [--no-cache] [--no-tty] [--no-mount] [--debug] <package>"
  echo "<package>:"
  for package in $_PACKAGES; do
    echo "  - $package"
  done
}

PACKAGE=""
FLAG_CLEAN=0
FLAG_PACKAGE=0
DOCKER_BUILD_FLAGS=""
DOCKER_MOUNT_TYPE=mount
DEBUG=0

while [ $# -ne 0 ]; do
  case "$1" in
    "--clean" ) FLAG_CLEAN=1 ;;
    "--package" ) FLAG_PACKAGE=1 ;;
    "--no-cache" ) DOCKER_BUILD_FLAGS="$DOCKER_BUILD_FLAGS --no-cache" ;;
    "--no-tty" ) DOCKER_BUILD_FLAGS="$DOCKER_BUILD_FLAGS --progress=plain" ;;
    "--no-mount" ) DOCKER_MOUNT_TYPE=nomount ;;
    "--debug" ) DEBUG=1 ;;
    --* )
      show_help
      exit 1
      ;;
    * )
      if [ -n "$PACKAGE" ]; then
        show_help
        exit 1
      fi
      PACKAGE="$1"
      ;;
  esac
  shift 1
done

DOCKER_PLATFORM=""
if [ "`uname -sm`" = "Darwin arm64" ]; then
  # M1 Mac の場合は --platform を指定する
  DOCKER_PLATFORM="--platform=linux/amd64"
fi

_FOUND=0
for package in $_PACKAGES; do
  if [ "$PACKAGE" = "$package" ]; then
    _FOUND=1
    break
  fi
done

if [ $_FOUND -eq 0 ]; then
  show_help
  exit 1
fi

echo "--clean: " $FLAG_CLEAN
echo "--package: " $FLAG_PACKAGE
echo "<package>: " $PACKAGE

set -ex

pushd ..
  MOMO_COMMIT="`git rev-parse HEAD`"
  MOMO_COMMIT_SHORT="`cat $MOMO_COMMIT | cut -b 1-8`"
popd

source ../VERSION

case "$PACKAGE" in
  "windows_x86_64" )
    echo "Windows では build.bat を利用してください。"
    exit 1
    ;;
  macos_arm64 )
    if [ $FLAG_CLEAN -eq 1 ]; then
      rm -rf ../_build/$PACKAGE
      rm -rf $PACKAGE/_source
      rm -rf $PACKAGE/_build
      rm -rf $PACKAGE/_install
      exit 0
    fi

    ./$PACKAGE/install_deps.sh

    source ./$PACKAGE/_install/webrtc/VERSIONS

    if [ -z "$JOBS" ]; then
      JOBS=`sysctl -n hw.logicalcpu_max`
      if [ -z "$JOBS" ]; then
        JOBS=1
      fi
    fi

    if [ $DEBUG -eq 1 ]; then
      BUILD_TYPE=Debug
    else
      BUILD_TYPE=Release
    fi

    mkdir -p ../_build/$PACKAGE
    pushd ../_build/$PACKAGE
      cmake \
        -DCMAKE_BUILD_TYPE=$BUILD_TYPE \
        -DMOMO_PACKAGE_NAME="$PACKAGE" \
        -DMOMO_VERSION="$MOMO_VERSION" \
        -DMOMO_COMMIT="$MOMO_COMMIT" \
        -DWEBRTC_BUILD_VERSION="$WEBRTC_BUILD_VERSION" \
        -DWEBRTC_READABLE_VERSION="$WEBRTC_READABLE_VERSION" \
        -DWEBRTC_COMMIT="$WEBRTC_COMMIT" \
        ../..
      cmake --build . -j$JOBS
    popd

    if [ $FLAG_PACKAGE -eq 1 ]; then
      MACOS_VERSION=`sw_vers -productVersion | cut -d '.' -f-2`

      pushd ..
        # パッケージのバイナリを作る
        MACOS_ARCH=${PACKAGE#"macos_"}
        rm -rf _package/momo-${MOMO_VERSION}_macos-${MACOS_VERSION}_${MACOS_ARCH}
        rm -f _package/momo-${MOMO_VERSION}_macos-${MACOS_VERSION}_${MACOS_ARCH}.tar.gz
        mkdir -p _package/momo-${MOMO_VERSION}_macos-${MACOS_VERSION}_${MACOS_ARCH}
        cp    _build/${PACKAGE}/momo _package/momo-${MOMO_VERSION}_macos-${MACOS_VERSION}_${MACOS_ARCH}/
        cp    LICENSE                _package/momo-${MOMO_VERSION}_macos-${MACOS_VERSION}_${MACOS_ARCH}/
        cp    NOTICE                 _package/momo-${MOMO_VERSION}_macos-${MACOS_VERSION}_${MACOS_ARCH}/
        cp -r html                   _package/momo-${MOMO_VERSION}_macos-${MACOS_VERSION}_${MACOS_ARCH}/html
        pushd _package
          tar czf momo-${MOMO_VERSION}_macos-${MACOS_VERSION}_${MACOS_ARCH}.tar.gz momo-${MOMO_VERSION}_macos-${MACOS_VERSION}_${MACOS_ARCH}
        popd

        rm -rf _package/momo-${MOMO_VERSION}_macos-${MACOS_VERSION}_${MACOS_ARCH}
        echo ""
        echo "パッケージが _package/momo-${MOMO_VERSION}_macos-${MACOS_VERSION}_${MACOS_ARCH}.tar.gz に生成されました。"
      popd
    fi

    ;;
  * )
    if [ $FLAG_CLEAN -eq 1 ]; then
      rm -rf ../_build/$PACKAGE
      IMAGES="`docker image ls -q momo/$PACKAGE`"
      if [ -n "$IMAGES" ]; then
        docker image rm $IMAGES
      fi
      docker builder prune -f --filter=label=jp.shiguredo.momo=$PACKAGE
      exit 0
    fi

    rm -rf $PACKAGE/script
    cp -r ../script $PACKAGE/script
    rm -rf $PACKAGE/patch
    cp -r ../patch $PACKAGE/patch

    # 可能な限りキャッシュを利用する
    mkdir -p $PACKAGE/_cache/boost/
    if [ -e ../_cache/boost/ ]; then
      cp -r ../_cache/boost/* $PACKAGE/_cache/boost/
    fi

    DOCKER_BUILDKIT=1 docker build $DOCKER_PLATFORM \
      -t momo/$PACKAGE:m$WEBRTC_BUILD_VERSION \
      $DOCKER_BUILD_FLAGS \
      --build-arg WEBRTC_BUILD_VERSION=$WEBRTC_BUILD_VERSION \
      --build-arg BOOST_VERSION=$BOOST_VERSION \
      --build-arg SDL2_VERSION=$SDL2_VERSION \
      --build-arg CLI11_VERSION=$CLI11_VERSION \
      --build-arg CMAKE_VERSION=$CMAKE_VERSION \
      --build-arg CUDA_VERSION=$CUDA_VERSION \
      --build-arg LIBVA_VERSION=$LIBVA_VERSION \
      --build-arg MSDK_VERSION=$MSDK_VERSION \
      --build-arg PACKAGE_NAME=$PACKAGE \
      $PACKAGE

    rm -rf $PACKAGE/_cache/boost/

    # キャッシュしたデータを取り出す
    set +e
    docker $DOCKER_PLATFORM container create -it --name momo-$PACKAGE momo/$PACKAGE:m$WEBRTC_BUILD_VERSION
    docker container start momo-$PACKAGE
    mkdir -p ../_cache/boost/
    docker container cp momo-$PACKAGE:/root/_cache/boost/. ../_cache/boost/
    docker container stop momo-$PACKAGE
    docker container rm momo-$PACKAGE
    set -e

    rm -r $PACKAGE/script
    rm -r $PACKAGE/patch

    ../script/docker_run.sh `pwd` `pwd`/.. $DOCKER_MOUNT_TYPE $PACKAGE momo/$PACKAGE:m$WEBRTC_BUILD_VERSION $MOMO_COMMIT

    if [ $FLAG_PACKAGE -eq 1 ]; then
      pushd ..
        rm -rf _package/momo-${MOMO_VERSION}_${PACKAGE}
        rm -f _package/momo-${MOMO_VERSION}_${PACKAGE}.tar.gz
        mkdir -p _package/momo-${MOMO_VERSION}_${PACKAGE}
        cp    _build/${PACKAGE}/momo _package/momo-${MOMO_VERSION}_${PACKAGE}/
        cp    LICENSE                _package/momo-${MOMO_VERSION}_${PACKAGE}/
        cp    NOTICE                 _package/momo-${MOMO_VERSION}_${PACKAGE}/
        cp -r html                   _package/momo-${MOMO_VERSION}_${PACKAGE}/html
        if [ -e _build/${PACKAGE}/libcamerac.so ]; then
          cp _build/${PACKAGE}/libcamerac.so _package/momo-${MOMO_VERSION}_${PACKAGE}/
        fi
        pushd _package
          tar czf momo-${MOMO_VERSION}_${PACKAGE}.tar.gz momo-${MOMO_VERSION}_${PACKAGE}
        popd

        rm -rf _package/momo-${MOMO_VERSION}_${PACKAGE}
        echo ""
        echo "パッケージが _package/momo-${MOMO_VERSION}_${PACKAGE}.tar.gz に生成されました。"
      popd
    fi
    ;;
esac
