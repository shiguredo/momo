# パッケージを作成する

## Raspberry Pi OS 32bit (armv6) 向けパッケージの作成例

raspberry-pi-os_armv6 パッケージを作成する場合は build ディレクトリで ./build.sh --package raspberry-pi-os_armv6 と打つことでパッケージが作成されます。

```shell
$ ./build.sh --package raspberry-pi-os_armv6
```

Windows の場合は以下のコマンドになります。

```powershell
.\build.bat -package
```

パッケージは `_package` 以下に作成されます。

### 作成可能なパッケージ一覧

- windows
- macos_arm64
- raspberry-pi-os_armv6
- raspberry-pi-os_armv7
- raspberry-pi-os_armv8
- ubuntu-18.04_armv8_jetson_nano
- ubuntu-18.04_armv8_jetson_xavier
- ubuntu-20.04_x86_64

## パッケージ解凍後の構成

```
$ tree
.
├── html
│   ├── test.html
│   └── webrtc.js
├── LICENSE
├── momo
└── NOTICE
```
