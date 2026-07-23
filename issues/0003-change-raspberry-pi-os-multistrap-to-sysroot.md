# Raspberry Pi OS の multistrap を署名検証付き sysroot に切り替える

- Priority: Medium
- Created: 2026-07-23
- Completed: {YYYY-MM-DD}
- Model: Composer 2.5
- Branch: feature/change-raspberry-pi-os-multistrap-to-sysroot
- Polished: {YYYY-MM-DD}

## 目的

`raspberry-pi-os_armv8` の rootfs 生成を、insecure な `multistrap` + HTTP + `--no-auth` から、webrtc-build / sora-cpp-sdk / sora-python-sdk と同系の署名検証付き sysroot builder（HTTPS + `signed_by`）へ切り替える。

Jetson (`ubuntu-22.04_armv8_jetson`) は本 issue の対象外とし、既存の `multistrap` 経路を残す。Jetson 側の移行は [0004-change-jetson-multistrap-to-sysroot.md](./0004-change-jetson-multistrap-to-sysroot.md) で扱う。

## 優先度根拠

Medium とする。

- CI は Ubuntu 22.04 上で `multistrap` をまだ利用できるため、即座のブロッカーではない。
- 一方で CI が `/usr/sbin/multistrap` を `sed` で書き換え、`Acquire::AllowInsecureRepositories=true` を注入している（`.github/workflows/build.yml` L135-141）。
- `multistrap` 自体は Debian unstable から削除済みで、将来の runner 更新の障害になる。
- Jetson を同時に扱わず Raspberry Pi OS だけを先に切り替えることで、リスクを分割する。

## 現状

### 呼び出し経路

- `run.py` L62-74 の `install_deps()` が `platform.target.os == "jetson" or platform.target.os == "raspberry-pi-os"` のとき `install_rootfs()` を呼ぶ。
- conf パスは `multistrap/{platform.target.package_name}.conf`。
- conf の MD5 を `install_dir/rootfs.version` に保存し、`@versioned` でキャッシュ判定する。
- `buildbase.py` L1027-1071 の `install_rootfs()` は `multistrap --no-auth -a arm64` を実行し、絶対 symlink の相対化と Jetson 向け `libnvbuf_fdmap.so` symlink 補正を行う。

### Raspberry Pi OS 用 conf

`multistrap/raspberry-pi-os_armv8.conf`:

- suite は既に `trixie`、`libstdc++-14-dev` を使用している。
- Debian 側: `http://deb.debian.org/debian`（HTTP）。
- Raspberry Pi 側: `http://archive.raspberrypi.org/debian`（HTTP）。
- packages: `libc6-dev` / `libstdc++-14-dev` / `libasound2-dev` / `libpulse-dev` / `libudev-dev` / `libexpat1-dev` / `libnss3-dev` / `libxext-dev` / `libxtst-dev` / `libcamera-dev`。

### CI

`.github/workflows/build.yml` の `build-ubuntu` matrix に `raspberry-pi-os_armv8` と `ubuntu-22.04_armv8_jetson` がある。両者共通の step（L135-141）で `multistrap` install と insecure sed を実行している。

`AVAILABLE_TARGETS`（`run.py` L411-419）と CMake / Boost / SDL3 の cross 経路は `install_dir/rootfs` を `CMAKE_SYSROOT` として参照する。rootfs の配置先は変えない。

## スコープ

含む:

1. `sysroot_builder.py` をリポジトリルートに追加する（module + CLI 一体）。
2. `sysroot/raspberry-pi-os_armv8.json` と `sysroot/keyrings/`（Debian / Raspberry Pi の keyring）を追加する。
3. `RepositoryConfig` に optional `pin_priority` を含め、Raspberry Pi repository に `pin_priority: 990` を付ける（`libcamera-dev` 等の overlay を Debian より優先するため）。
4. `run.py` に `install_sysroot()` を追加し、`raspberry-pi-os` だけ sysroot builder を呼ぶ。Jetson は従来どおり `install_rootfs()` を呼ぶ。
5. `multistrap/raspberry-pi-os_armv8.conf` を削除する。Jetson conf は残す。
6. `.github/workflows/build.yml` の deps step を target 別に分割し、Raspberry Pi OS から `multistrap` / insecure sed を除去する。Jetson 側は現行のまま残す。
7. `sysroot_builder` のユニットテストを追加する（ネットワーク・モック・スタブ禁止）。
8. `CHANGES.md` の `## develop` に `[CHANGE]` エントリを追記する。

含まない:

- Jetson（`ubuntu-22.04_armv8_jetson` / `multistrap/ubuntu-22.04_armv8_jetson.conf` / `install_rootfs()` の Jetson 固有副作用）。0004 で扱う。
- `buildbase.py` からの `install_rootfs()` 削除（Jetson が使うため残す）。
- generic Ubuntu armv8 ターゲットの追加。
- CI job 間の sysroot cache。
- macOS ホスト上での sysroot 生成対応（Linux ホスト限定でよい）。

## 設計方針

### 移植元

- canonical は sora-python-sdk の `sysroot_builder.py`（`pin_priority` 付き）。sora-cpp-sdk / webrtc-build の `feature/sysroot` は参考にする。
- 移植時の参照コミットは実装着手時に sora-python-sdk の `develop` 先端（または該当マージコミット）を pin し、PR 本文に記録する。
- manifest 名は `.webrtc-build-sysroot.json` を維持する（cross-repo 互換）。

### JSON

`sysroot/raspberry-pi-os_armv8.json`:

```json
{
    "name": "raspberry-pi-os_armv8",
    "arch": "arm64",
    "triplet": "aarch64-linux-gnu",
    "packages": [
        "libc6-dev",
        "libstdc++-14-dev",
        "libasound2-dev",
        "libpulse-dev",
        "libudev-dev",
        "libexpat1-dev",
        "libnss3-dev",
        "libxext-dev",
        "libxtst-dev",
        "libcamera-dev"
    ],
    "repositories": [
        {
            "url": "https://deb.debian.org/debian",
            "suite": "trixie",
            "components": ["main"],
            "signed_by": "keyrings/debian-archive-keyring.gpg"
        },
        {
            "url": "https://archive.raspberrypi.com/debian",
            "suite": "trixie",
            "components": ["main"],
            "signed_by": "keyrings/raspberrypi-archive-keyring.asc",
            "pin_priority": 990
        }
    ]
}
```

- packages 集合は現行 conf と同一とする（suite も既に trixie）。
- keyring は sora-python-sdk の `sysroot/keyrings/` から移植し、SHA-256 を PR 作成時に照合する。
  - `debian-archive-keyring.gpg`: `506b815cbb32d9b6066b4a2aa524071e071761e7e7f68c3ac74f3061ba852017`
  - `raspberrypi-archive-keyring.asc`: `76603890d82a492175caf17aba68dc73acb1189c9fd58ec0c19145dfa3866d56`

### run.py への接続

`buildbase.py` は melpon/buildbase テンプレート（`curl -LO` 上書き想定）のため、`install_sysroot()` は `run.py` 側に置く。

擬似コード:

```python
if platform.target.os == "jetson":
    conf = os.path.join(BASE_DIR, "multistrap", f"{platform.target.package_name}.conf")
    version_md5 = hashlib.md5(open(conf, "rb").read()).hexdigest()
    install_rootfs(
        version=version_md5,
        version_file=os.path.join(install_dir, "rootfs.version"),
        install_dir=install_dir,
        conf=conf,
        arch="arm64",
    )
elif platform.target.os == "raspberry-pi-os":
    config_path = os.path.join(BASE_DIR, "sysroot", f"{platform.target.package_name}.json")
    install_sysroot(config_path=config_path, install_dir=install_dir)
```

- `install_sysroot()` は `sys.executable` で `sysroot_builder.py --config ... --dest <install_dir>/rootfs` を呼ぶ。
- `@versioned` / `rootfs.version` は Raspberry Pi OS では使わない。再利用判定は builder 側 manifest に一本化する。
- 既存の `_install/raspberry-pi-os_armv8/*/rootfs` があるローカル環境は、初回 build 前に rootfs と `rootfs.version` を手動削除する必要がある（由来不明 rootfs は `--force` 無しで拒否される）。この注意は `CHANGES.md` に書く。

### CI

`.github/workflows/build.yml` L135-141 を target 別に分割する。

- `ubuntu-22.04_armv8_jetson`: 現行どおり `multistrap` + insecure sed + `binutils-aarch64-linux-gnu`。
- `raspberry-pi-os_armv8`: `binutils-aarch64-linux-gnu` のみ。vendored keyring の SHA-256 検証 step を追加する。`multistrap` / sed は入れない。
- Raspberry Pi OS build 後に代表ヘッダと manifest の存在確認 step を追加する（`libcamera/camera.h`、`c++/14/vector`、`.webrtc-build-sysroot.json` の `deb_files` 件数など）。

### テスト

- `tests/sysroot_builder/test_sysroot_builder.py`（配置は実装時に既存 `test/` との衝突を避けて決める）で、validation・fingerprint・symlink 相対化・manifest 再利用 / 拒否・`pin_priority` をネットワーク無しで検証する。
- 統合確認は `python3 run.py build raspberry-pi-os_armv8`（CI の `build-ubuntu` matrix）で行う。
- Jetson / Windows / macOS / Ubuntu x86_64 に回帰が無いこと。

## 完了条件

- `multistrap/raspberry-pi-os_armv8.conf` が削除されている。
- `sysroot_builder.py` と `sysroot/raspberry-pi-os_armv8.json`、keyring 2 件が追加されている。
- `run.py` が Raspberry Pi OS で `install_sysroot()`、Jetson で `install_rootfs()` を呼ぶ。
- CI の Raspberry Pi OS entry から `multistrap` / insecure sed が消えている。Jetson entry には残っている。
- `python3 run.py build raspberry-pi-os_armv8` が成功する。
- sysroot builder のユニットテストが通る。
- `CHANGES.md` の `## develop` にエントリがある。
- 0004（Jetson）が本 issue の成果物を前提に書ける状態になっている。

## 参考

- sora-python-sdk `issues/closed/0074-change-multistrap-to-sysroot.md`
- sora-cpp-sdk `issues/closed/0002-change-replace-multistrap-with-sysroot.md`
- webrtc-build `feature/sysroot`（`sysroot_builder.py`）
- 後続: [0004-change-jetson-multistrap-to-sysroot.md](./0004-change-jetson-multistrap-to-sysroot.md)
