#!/bin/bash

set -e

# ヘルプ表示
function show_help() {
  echo ""
  echo "$0 <作業ディレクトリ> <Momoリポジトリのルートディレクトリ> <マウントタイプ [mount | nomount]> <パッケージ名> <Dockerイメージ名> <MOMO_COMMIT>"
  echo ""
}

# 引数のチェック
if [ $# -ne 6 ]; then
  show_help
  exit 1
fi

WORK_DIR="$1"
MOMO_DIR="$2"
MOUNT_TYPE="$3"
PACKAGE_NAME="$4"
DOCKER_IMAGE="$5"
MOMO_COMMIT="$6"

if [ -z "$WORK_DIR" ]; then
  echo "エラー: <作業ディレクトリ> が空です"
  show_help
  exit 1
fi

if [ -z "$MOMO_DIR" ]; then
  echo "エラー: <Momoリポジトリのルートディレクトリ> が空です"
  show_help
  exit 1
fi

if [ ! -e "$MOMO_DIR/.git" ]; then
  echo "エラー: $MOMO_DIR は Git リポジトリのルートディレクトリではありません"
  show_help
  exit 1
fi

if [ "$MOUNT_TYPE" != "mount" -a "$MOUNT_TYPE" != "nomount"  ]; then
  echo "エラー: <マウントタイプ> は mount または nomount である必要があります"
  show_help
  exit 1
fi

if [ -z "$PACKAGE_NAME" ]; then
  echo "エラー: <パッケージ名> が空です"
  show_help
  exit 1
fi

if [ -z "$(docker images -q $DOCKER_IMAGE)" ]; then
  echo "エラー: <Dockerイメージ名> $DOCKER_IMAGE が存在しません"
  show_help
  exit 1
fi

if [ -z "$MOMO_COMMIT" ]; then
  echo "エラー: <MOMO_COMMIT> が空です"
  show_help
  exit 1
fi

if [ ! -e "$MOMO_DIR/VERSION" ]; then
  echo "エラー: $MOMO_DIR/VERSION が存在しません"
  exit 1
fi

source $MOMO_DIR/VERSION

# マウントするかどうかで大きく分岐する
if [ "$MOUNT_TYPE" = "mount" ]; then
  # マウントする場合は、単純にマウントしてビルドするだけ
  docker run \
    -it \
    --rm \
    -v "$WORK_DIR/..:/root/momo" \
    "$DOCKER_IMAGE" \
    /bin/bash -c "
      set -ex
      mkdir -p /root/momo/_build/$PACKAGE_NAME
      cd /root/momo/_build/$PACKAGE_NAME
      cmake \
        -DCMAKE_BUILD_TYPE=Release \
        -DMOMO_PACKAGE_NAME=$PACKAGE_NAME \
        -DMOMO_VERSION=$MOMO_VERSION \
        -DMOMO_COMMIT=$MOMO_COMMIT \
        -DWEBRTC_BUILD_VERSION=$WEBRTC_BUILD_VERSION \
        -DWEBRTC_READABLE_VERSION=$WEBRTC_READABLE_VERSION \
        -DWEBRTC_COMMIT=$WEBRTC_COMMIT \
        ../..
      VERBOSE=$VERBOSE cmake --build . -j\$(nproc)
    "
else
  # マウントしない場合は、コンテナを起動して、コンテナに必要なファイルを転送して、コンテナ上でビルドして、生成されたファイルをコンテナから戻して、コンテナを終了する

  pushd $MOMO_DIR
  if git diff-index --quiet HEAD --; then
    :
  else
    # ローカルの変更があるので確認する
    git status
    read -p "ローカルの変更があります。これらの変更はビルドに反映されません。続行しますか？ (y/N): " yn
    case "$yn" in
      [yY]*)
        ;;
      *)
        exit 1
        ;;
    esac
  fi
  popd

  pushd $WORK_DIR

  # 途中でエラーが起きても確実にコンテナを後片付けする
  trap "set +e; docker container stop momo-$PACKAGE_NAME; docker container rm momo-$PACKAGE_NAME" 0

  # ベースイメージから構築したコンテナに転送してビルドし、
  # ビルドが完了したら成果物や中間ファイルを取り出す
  docker container create -it --name momo-$PACKAGE_NAME "$DOCKER_IMAGE"
  docker container start momo-$PACKAGE_NAME

  # 転送用の momo のソースを生成（中間ファイルも含める）
  rm -rf momo
  git clone $MOMO_DIR momo

  # 中間ファイルのコピー
  mkdir -p $MOMO_DIR/_build
  if [ -e $MOMO_DIR/_build/$PACKAGE_NAME ]; then
    mkdir -p momo/_build
    cp -r $MOMO_DIR/_build/$PACKAGE_NAME momo/_build/$PACKAGE_NAME
  fi

  # 更新日時を元ファイルに合わせる
  pushd momo
  find . -type f | while read file; do
    if [ -e "$MOMO_DIR/$file" ]; then
      # -c: ファイルを生成しない
      # -m: 更新日時を更新
      # -r <file>: このファイルの日時に合わせる
      touch -c -m -r "$MOMO_DIR/$file" "$file"
    fi
  done
  popd

  tar czf momo.tar.gz momo
  rm -rf momo

  # ソースを転送して Docker の中でビルドする
  docker container cp momo.tar.gz momo-$PACKAGE_NAME:/root/
  rm momo.tar.gz

  docker container exec momo-$PACKAGE_NAME /bin/bash -c 'cd /root && tar xf momo.tar.gz && rm momo.tar.gz'
  docker run \
    -it \
    --rm \
    -v "$WORK_DIR/..:/root/momo" \
    "$DOCKER_IMAGE" \
    /bin/bash -c "
      set -ex
      mkdir -p /root/momo/_build/$PACKAGE_NAME
      cd /root/momo/_build/$PACKAGE_NAME
      cmake \
        -DCMAKE_BUILD_TYPE=Release \
        -DMOMO_PACKAGE_NAME=$PACKAGE_NAME \
        -DMOMO_VERSION=$MOMO_VERSION \
        -DMOMO_COMMIT=$MOMO_COMMIT \
        -DWEBRTC_BUILD_VERSION=$WEBRTC_BUILD_VERSION \
        -DWEBRTC_READABLE_VERSION=$WEBRTC_READABLE_VERSION \
        -DWEBRTC_COMMIT=$WEBRTC_COMMIT \
        ../..
      VERBOSE=$VERBOSE cmake --build . -j\$(nproc)
    "

  # 中間ファイル類を取り出す
  rm -rf $MOMO_DIR/_build/$PACKAGE_NAME
  docker container cp momo-$PACKAGE_NAME:/root/momo/_build/$PACKAGE_NAME/ $MOMO_DIR/_build/$PACKAGE_NAME

  popd
fi
