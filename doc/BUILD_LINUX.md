# Linux 版 Momo をビルドする

まずは momo のリポジトリをダウンロードします。

```shell
$ git clone git@github.com:shiguredo/momo.git
```

## Docker の利用について

Linux 版 Momo をビルドする際には Docker 19.03 以降が必要になりますので、事前にインストールしておいてください。

Docker for Windows では未検証です。Linux 版、または Docker for Mac をご利用ください。

また、make コマンド実行時に NOMOUNT=1 オプションを指定することで、
マウントを利用しないモードで docker container を動作させることができます。何らかの理由でマウントがうまく動作しない場合に使って下さい。

## Raspbian (armv6) 向けバイナリを作成する

build ディレクトリ以下で make raspbian-buster_armv6 と打つことで Momo のバイナリが生成されます。

```shell
$ make raspbian-buster_armv6
```

うまくいかない場合は `make raspbian-buster_armv6.clean && make raspbian-buster_armv6` を試してみてください。それでもだめな場合は issues にお願いします。

## Raspbian (armv7) 向けバイナリを作成する

Raspberry Pi 3 B/B+ は実際は armv8 ですが 64 ビット機能が Raspbian では利用できないため、実質 armv7 相当のビルドになります。

build ディレクトリ以下で make raspbian-buster_armv7 と打つことで Momo のバイナリが生成されます。

```shell
$ make raspbian-buster_armv7
```

うまくいかない場合は `make raspbian-buster_armv7.clean && make raspbian-buster_armv7` を試してみてください。それでもだめな場合は issues にお願いします。

## Ubuntu 18.04 (armv8) 向けバイナリを作成する

build ディレクトリ以下で make ubuntu-18.04_armv8 と打つことで Momo のバイナリが生成されます。

```shell
$ make ubuntu-18.04_armv8
```

うまくいかない場合は `make ubuntu-18.04_armv8.clean && make ubuntu-18.04_armv8` を試してみてください。それでもだめな場合は issues にお願いします。

## Ubuntu 18.04 (armv8) Jetson Nano 向けバイナリを作成する

build ディレクトリ以下で make ubuntu-18.04_armv8 と打つことで Momo のバイナリが生成されます。

```shell
$ make ubuntu-18.04_armv8_jetson_nano
```

うまくいかない場合は `make ubuntu-18.04_armv8_jetson_nano.clean && make ubuntu-18.04_armv8_jetson_nano` を試してみてください。それでもだめな場合は issues にお願いします。

## Ubuntu 18.04 (x86_64) 向けバイナリを作成する

build ディレクトリ以下で make ubuntu-18.04_x86_64 と打つことで Momo の Ubuntu 18.04 x86_64 向けバイナリが生成されます。

```shell
$ make ubuntu-18.04_x86_64
```

うまくいかない場合は `make ubuntu-18.04_x86_64.clean && make ubuntu-18.04_x86_64` を試してみてください。それでもだめな場合は issues にお願いします。


ubuntu-16.04_armv7_ros


## Ubuntu 16.04 (armv7) 向け ROS 対応バイナリを作成する

build ディレクトリ以下で make ubuntu-16.04_armv7_ros と打つことで Momo の Ubuntu 18.04 x86_64 向けバイナリが生成されます。

```shell
$ make ubuntu-16.04_armv7_ros
```

うまくいかない場合は `make ubuntu-16.04_armv7_ros.clean && make ubuntu-16.04_armv7_ros` を試してみてください。それでもだめな場合は issues にお願いします。


## Ubuntu 16.04 (armv7) 向け ROS 対応バイナリを作成する

build ディレクトリ以下で make ubuntu-16.04_x86_64_ros と打つことで Momo の Ubuntu 18.04 x86_64 向けバイナリが生成されます。

```shell
$ make ubuntu-16.04_x86_64_ros
```

うまくいかない場合は `make ubuntu-16.04_x86_64_ros.clean && make ubuntu-16.04_x86_64_ros` を試してみてください。それでもだめな場合は issues にお願いします。
