# Linux でビデオデバイスを指定する

Momo ではデバイスを指定する際、デバイスファイルではなくデバイス名を指定します。

## --video-device

`--video-device` は linux 端末でビデオデバイス（つまりカメラ）を指定する機能です。
1 台の Raspberry Pi で複数の Momo を起動し、ビデオデバイスが複数あり、それぞれここに割り当てたい時に利用できます。

```bash
./momo --video-device "MX Brio" test
```

### デバイス名を取得する

デバイス名は、`v4l2-ctl` コマンドで取得できます。
例えば以下のような実行結果の場合、デバイス名は `HD USB CAMERA: HD USB CAMER` と `Integrated_Webcam: Integrate` です。

```bash
v4l2-ctl --list-devices
HD USB CAMERA: HD USB CAMERA (usb-0000:00:00.0-1):
        /dev/video0
        /dev/video1
        /dev/media0

Integrated_Webcam: Integrate (usb-0000:00:00.0-12):
        /dev/video2
        /dev/video3
        /dev/media1
```

### デバイス名の固定

Linux 端末で USB カメラを接続した場合に、デバイス名が前回の接続時と異なるデバイス名になることがあります。
デバイス名が変更された場合は、 `--video-device` に指定するデバイス名も合わせて修正する必要があるため、
例えば、Linux 端末の起動時に自動で momo を起動させたい場合等に不便です。

ここでは、同じデバイス名を使用できるように、USB カメラのデバイス名を接続するカメラごとに固定して、
同一の USB カメラでは常に同じデバイス名を使用できるように設定します。

#### udev ルールの設定

udev ルールを設定してデバイス名を固定します。

まず、接続した USB カメラの SERIAL を取得します

```console
$ udevadm info --query=all --name=/dev/video0 | grep ID_SERIAL_SHORT
E: ID_SERIAL_SHORT=8E40F950
```

次に、/etc/udev/rules.d/ 以下にルールファイルを作成（例: /etc/udev/rules.d/70-usb.rules）して下記のように設定します。

ATTRS{serial} には上記で取得した SERIAL を指定します。

SYMLINK には固定するデバイス名を指定します。

```bash
KERNEL=="video[0-9]*", MODE="0666", ATTRS{serial}=="8E40F950", SYMLINK+="video_101"
```

#### デバイス名が複数ある場合

1 つの USB カメラにデバイス名が複数ある場合は、指定するデバイスの ATTR{index} も udev ルールに指定します。

ATTR{index} は下記の方法で取得します。

```bash
udevadm info --attribute-walk --path $( udevadm info --query=path --name=/dev/video0 ) | grep index ATTR{index}=="0"
```

ATTR{index}=="0" を含めた場合の設定例は下記の通りです。

```bash
KERNEL=="video[0-9]*", MODE="0666", ATTRS{serial}=="8E40F950", ATTR{index}=="0", SYMLINK+="video_101"
```

#### momo の実行

設定したデバイス名を使用するために、USB カメラを Linux 端末から外して、
すぐに再度接続して、上記で SYMLINK に設定したデバイス名を --video-device に指定して momo を実行します。

```bash
./momo --video-device "video_101" test
```
