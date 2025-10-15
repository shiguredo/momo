# macOS で Momo を使ってみる

## macOS 向けのバイナリは以下にて提供しています

<https://github.com/shiguredo/momo/releases> にて最新版のバイナリをダウンロードしてください。

## 動かしてみる

動かし方については [USE_P2P.md](USE_P2P.md) を御覧ください。

## --video-input-device

`--video-input-device` は macOS でビデオデバイス（つまりカメラ）を指定する機能です。 1 台の macOS で複数の Momo を起動し、ビデオデバイスが複数あり、それぞれ個々に割り当てたい時に利用できます。

```bash
./momo --video-input-device 0 p2p
```

### デバイス一覧を取得する

```bash
system_profiler SPCameraDataType
```

### ビデオデバイスの指定方法

ビデオデバイスの指定には、デバイス番号、またはデバイス名を指定することができます。

- 指定がない場合は、デバイス番号が 0 のものが選択されます
- デバイス番号 0 のエイリアスとして、`default` が使えます
- デバイス名を指定した場合は、デバイス番号 0 から順番に検索していき、前方一致検索で最初にマッチしたものが選択されます

なお、デバイス番号は接続する順番によって変動するため、同じデバイス番号を指定しても異なるデバイスが選択されることがあります。
このため、使用するデバイスを固定したい場合は、デバイス名で指定するようにしてください。

#### デフォルトデバイス指定

下記は全て同じデフォルトデバイスが選択されます。

```console
./momo p2p
./momo --video-input-device 0 p2p
./momo --video-input-device default p2p
./momo --video-input-device "default" p2p
```

#### デバイス番号で指定

```console
./momo --video-input-device 1 p2p
```

#### デバイス名で指定

前方一致検索でマッチさせるため、下記は全て同じデバイスが選択されます。

```console
./momo --video-input-device FaceTime p2p
./momo --video-input-device "FaceTime HD" 1 p2p
./momo --video-input-device "FaceTime HD Camera (Built-in)" p2p
```
