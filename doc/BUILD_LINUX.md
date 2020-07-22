# Linux 版 Momo をビルドする

まずは momo のリポジトリをダウンロードします。

```shell
$ git clone git@github.com:shiguredo/momo.git
```

## Docker の利用について

Linux 版 Momo をビルドする際には Docker 19.03 以降が必要になりますので、事前にインストールしておいてください。

Docker for Windows では未検証です。Linux 版、または Docker for Mac をご利用ください。

また、./build.sh 実行時に --no-mount オプションを指定することで、
マウントを利用しないモードで docker container を動作させることができます。何らかの理由でマウントがうまく動作しない場合に使って下さい。

## Raspberry Pi OS 32bit (armv6) 向けバイナリを作成する

build ディレクトリ以下で ./build.sh raspberry-pi-os_armv6 と打つことで Momo の Raspberry Pi OS armv6 向けバイナリが生成されます。

```shell
$ ./build.sh raspberry-pi-os_armv6
```

うまくいかない場合は `./build.sh --clean raspberry-pi-os_armv6 && ./build.sh raspberry-pi-os_armv6` を試してみてください。それでもだめな場合は Discord にお願いします。

## Raspberry Pi OS 32bit (armv7) 向けバイナリを作成する

build ディレクトリ以下で ./build.sh raspberry-pi-os_armv7 と打つことで Momo の Raspberry-Pi-OS armv7 向けバイナリが生成されます。

```shell
$ ./build.sh raspberry-pi-os_armv7
```

うまくいかない場合は `./build.sh --clean raspberry-pi-os_armv7 && ./build.sh raspberry-pi-os_armv7` を試してみてください。それでもだめな場合は Discord にお願いします。

## Raspberry Pi OS 64bit (armv8) 向けバイナリを作成する

build ディレクトリ以下で ./build.sh raspberry-pi-os_armv8 と打つことで Momo の Raspberry Pi OS armv8 向けバイナリが生成されます。

```shell
$ ./build.sh raspberry-pi-os_armv8
```

うまくいかない場合は `./build.sh --clean raspberry-pi-os_armv8 && ./build.sh raspberry-pi-os_armv8` を試してみてください。それでもだめな場合は Discord にお願いします。

## Ubuntu 18.04 (armv8) Jetson Nano 向けバイナリを作成する

build ディレクトリ以下で ./build.sh ubuntu-18.04_armv8_jetson_nano と打つことで Momo の Ubuntu 18.04 armv8 Jetson Nano 向けバイナリが生成されます。

```shell
$ ./build.sh ubuntu-18.04_armv8_jetson_nano
```

うまくいかない場合は `./build.sh --clean ubuntu-18.04_armv8_jetson_nano && ./build.sh ubuntu-18.04_armv8_jetson_nano` を試してみてください。それでもだめな場合は Discord にお願いします。

## Ubuntu 18.04 (armv8) Jetson Xavier NX / AGX 向けバイナリを作成する

build ディレクトリ以下で ./build.sh ubuntu-18.04_armv8_jetson_xavier と打つことで Momo の Ubuntu 18.04 armv8 Jetson Xavier NX / AGX 向けバイナリが生成されます。

```shell
$ ./build.sh ubuntu-18.04_armv8_jetson_xavier
```

うまくいかない場合は `./build.sh --clean ubuntu-18.04_armv8_jetson_xavier && ./build.sh ubuntu-18.04_armv8_jetson_xavier` を試してみてください。それでもだめな場合は Discord にお願いします。

## Ubuntu 18.04 (x86_64) 向けバイナリを作成する

build ディレクトリ以下で ./build.sh ubuntu-18.04_x86_64 と打つことで Momo の Ubuntu 18.04 x86_64 向けバイナリが生成されます。

```shell
$ ./build.sh ubuntu-18.04_x86_64
```

うまくいかない場合は `./build.sh --clean ubuntu-18.04_x86_64 && ./build.sh ubuntu-18.04_x86_64` を試してみてください。それでもだめな場合は Discord にお願いします。


## Ubuntu 16.04 (x86_64) 向け ROS 対応バイナリを作成する

build ディレクトリ以下で ./build.sh ubuntu-16.04_x86_64_ros と打つことで Momo の Ubuntu 16.04 x86_64 ROS 向けバイナリが生成されます。

```shell
$ ./build.sh ubuntu-16.04_x86_64_ros
```

うまくいかない場合は `./build.sh --clean ubuntu-16.04_x86_64_ros && ./build.sh ubuntu-16.04_x86_64_ros` を試してみてください。それでもだめな場合は Discord にお願いします。
