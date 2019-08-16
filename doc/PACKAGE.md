# パッケージを作成する

## Raspbian June 2018 (armv6) 向けパッケージの作成例

raspbian-buster_armv6 パッケージを作成する場合は raspbian-buster_armv6.package と打つことでパッケージが作成されます。

```shell
$ make raspbian-buster_armv6.package
```

パッケージは build/package 以下に作成されます。

### 作成可能なパッケージ一覧

- raspbian-buster_armv6
- raspbian-buster_armv7
- ubuntu-18.04_armv8
- ubuntu-18.04_x86_64
- macos
- ubuntu-16.04_armv7_ros
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
