# パッケージを作成する

## Raspbian Buster (armv6) 向けパッケージの作成例

raspbian-buster_armv6 パッケージを作成する場合は build ディレクトリで ./build.sh --package raspbian-buster_armv6 と打つことでパッケージが作成されます。

```shell
$ ./build.sh --package raspbian-buster_armv6
```

Windows の場合は以下のコマンドになります。

```powershell
.\build.bat -package
```

パッケージは `_package` 以下に作成されます。

### 作成可能なパッケージ一覧

- windows
- macos
- raspbian-buster_armv6
- raspbian-buster_armv7
- ubuntu-18.04_armv8_jetson
- ubuntu-18.04_x86_64
- ubuntu-20.04_x86_64
- ubuntu-16.04_x86_64_ros

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
