# Raspberry Pi OS の multistrap を署名検証付き sysroot に切り替える

- Priority: Medium
- Created: 2026-07-23
- Completed: {YYYY-MM-DD}
- Model: Composer 2.5
- Branch: feature/change-raspberry-pi-os-multistrap-to-sysroot
- Polished: 2026-08-18

## 目的

`raspberry-pi-os_armv8` の rootfs 生成を、insecure な `multistrap` + HTTP + `--no-auth` から、webrtc-build / sora-cpp-sdk / sora-python-sdk と同系の署名検証付き sysroot builder（HTTPS + `signed_by`）へ切り替える。

Jetson (`ubuntu-22.04_armv8_jetson`) は本 issue の対象外とし、既存の `multistrap` 経路を残す。Jetson 側の移行は [0004-change-jetson-multistrap-to-sysroot.md](./0004-change-jetson-multistrap-to-sysroot.md) で扱う。

## 優先度根拠

Medium とする。

- CI は Ubuntu 22.04 上で `multistrap` をまだ利用できるため、即座のブロッカーではない。
- 一方で `.github/workflows/build.yml` の `Install deps for Jetson and Raspberry Pi OS series` step が `/usr/sbin/multistrap` を `sed` で書き換え、`Acquire::AllowInsecureRepositories=true` を注入している。
- `multistrap` は Debian 側でパッケージメンテナンスが縮小しており、将来的な Debian ベースの runner 更新でパッケージが利用不能になる懸念がある。
- Jetson を同時に扱わず Raspberry Pi OS だけを先に切り替えることで、リスクを分割する。

## 現状

### 呼び出し経路

- `run.py` の `install_deps()` が `platform.target.os == "jetson" or platform.target.os == "raspberry-pi-os"` のとき `install_rootfs()`（`buildbase.py`）を呼ぶ。
- conf パスは `multistrap/{platform.target.package_name}.conf`。
- conf の MD5 を `install_dir/rootfs.version` に保存し、`buildbase.py` の `@versioned` デコレータでキャッシュ判定する。
- `buildbase.py` の `install_rootfs()` は `multistrap --no-auth -a arm64` を実行し、絶対 symlink の相対化と Jetson 向け `libnvbuf_fdmap.so` symlink 補正（tegra / nvidia 両ディレクトリ対応）を行う。`--no-auth` オプションは CLI 側で強制しており、Raspberry Pi OS の `raspberry-pi-os_armv8.conf` 自体には `noauth=true` は含まれない（Jetson conf のみ持つ）。

### Raspberry Pi OS 用 conf

`multistrap/raspberry-pi-os_armv8.conf`:

- suite は既に `trixie`、`libstdc++-14-dev` を使用している。
- `[Deb]` セクション: `source=http://deb.debian.org/debian`（HTTP）。packages は `libc6-dev` / `libstdc++-14-dev` / `libasound2-dev` / `libpulse-dev` / `libudev-dev` / `libexpat1-dev` / `libnss3-dev` / `libxext-dev` / `libxtst-dev` の 9 個。
- `[Rasp]` セクション: `source=http://archive.raspberrypi.org/debian`（HTTP）。packages は `libcamera-dev` のみ。
- 現状は各パッケージの取得元がセクションで固定されており、APT pinning は使っていない。

### CI

`.github/workflows/build.yml` の `build-ubuntu` matrix に `raspberry-pi-os_armv8` と `ubuntu-22.04_armv8_jetson` がある。両者共通の `Install deps for Jetson and Raspberry Pi OS series` step で `multistrap` install と insecure sed を実行している。

`run.py` の `AVAILABLE_TARGETS` に `raspberry-pi-os_armv8` と `ubuntu-22.04_armv8_jetson` がある。クロスコンパイル対象 (`raspberry-pi-os` / `jetson`) の CMake / Boost / SDL3 の各インストール経路は `install_dir/rootfs` を `CMAKE_SYSROOT` として参照する。rootfs の配置先は変えない。

## スコープ

含む:

1. `sysroot_builder.py` をリポジトリルートに追加する（module + CLI 一体）。
2. `sysroot/raspberry-pi-os_armv8.json` と `sysroot/keyrings/`（Debian / Raspberry Pi の keyring 2 件と、それらの期待 SHA-256 を並べた `SHA256SUMS`）を追加する。
3. canonical に既に存在する `RepositoryConfig.pin_priority` を利用し、Raspberry Pi repository に `pin_priority: 990` を付ける。狙いは Debian ミラーにも存在する `libcamera-dev` について、Raspberry Pi Ltd が Raspberry Pi 向けにパッチした固有ビルドの方を優先取得すること。値の 990 は sora-python-sdk canonical と揃える（sysroot builder は target release を設定しない構成のため、default (500) を確実に上回れば実効効果は変わらない。501 以上なら成立するが canonical との整合を優先する）。同一パッケージが両ミラーに存在する場合も Raspberry Pi 側から取得される semantics になり、現行 conf のセクション固定とは取得元の解決経路が変わる。
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

- canonical は sora-python-sdk の `sysroot_builder.py`（`RepositoryConfig.pin_priority` を含む現行実装）。sora-cpp-sdk / webrtc-build の `feature/sysroot` は参考にする。
- 移植時の参照コミットは実装着手時に sora-python-sdk の `develop` 先端（または該当マージコミット）を pin し、PR 本文に記録する。
- 移植の粒度は「canonical の `sysroot_builder.py` をファイル単位でそのまま持ってくる」を第一とする。`_PIN_PRIORITY_MIN` / `_PIN_PRIORITY_MAX` / `_REPOSITORY_ALLOWED_KEYS` などの validation もそのまま踏襲し、独自実装で差分を作らない。
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

- packages 集合は現行 conf の `[Deb]` + `[Rasp]` を合わせた 10 個と同一とする。取得元はセクションで固定せず APT pinning に一任し、`libcamera-dev` は `pin_priority: 990` により Raspberry Pi ミラーから取得される（`libcamera-dev` は Debian trixie にも存在するが、Raspberry Pi Ltd 固有のパッチ版が必要なため優先取得の対象）。
- Raspberry Pi ミラーの host は現行 conf の `archive.raspberrypi.org` から `archive.raspberrypi.com` に切り替える。`archive.raspberrypi.com` が Raspberry Pi Ltd の現在の canonical host であり、sora-python-sdk / sora-cpp-sdk の canonical JSON も `.com` を採用している。HTTPS + `signed_by` に揃える機会に `.com` へ統一する。
- keyring は sora-python-sdk の `sysroot/keyrings/` から移植し、sora-python-sdk 側の SHA-256 と PR 作成時に一致確認する。一次配布元は Debian の `debian-archive-keyring` パッケージと Raspberry Pi Ltd の `raspberrypi-archive-keyring` パッケージ。
  - `debian-archive-keyring.gpg`: `506b815cbb32d9b6066b4a2aa524071e071761e7e7f68c3ac74f3061ba852017`
  - `raspberrypi-archive-keyring.asc`: `76603890d82a492175caf17aba68dc73acb1189c9fd58ec0c19145dfa3866d56`

### run.py への接続

`buildbase.py` は melpon/buildbase テンプレート（`buildbase.py` の先頭コメントで由来 URL と `curl -LO` 上書きの手順を明記）で、テンプレート外の関数を足すと次回上書き時に消える。追加する `install_sysroot()` は `run.py` 側に置く。

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
- `install_sysroot()` は `force` パラメータを持たず常に false 扱いとする（`run.py build` の CLI からも露出させない）。
- `@versioned` / `rootfs.version` は Raspberry Pi OS では使わない。再利用判定は builder 側 manifest に一本化する。
- 旧 multistrap 由来の `_install/raspberry-pi-os_armv8/*/rootfs` があるローカル環境は、初回 build 前に `rootfs` ディレクトリを手動削除する必要がある（由来不明 rootfs は `--force` 無しで拒否される）。`rootfs.version` は新経路では読まれず残置しても副作用はないが、混乱を避けるため合わせて削除するのが望ましい。この注意は `CHANGES.md` に書く。

### CI

`.github/workflows/build.yml` の `Install deps for Jetson and Raspberry Pi OS series` step を target 別に分割する。

- `ubuntu-22.04_armv8_jetson`: 現行どおり `multistrap` + insecure sed + `binutils-aarch64-linux-gnu`。
- `raspberry-pi-os_armv8`: `binutils-aarch64-linux-gnu` のみ。vendored keyring の SHA-256 検証 step は `sysroot/keyrings/SHA256SUMS` に期待値を並べ、`sha256sum -c` で検証する形式に寄せる。`multistrap` / sed は入れない。
- Raspberry Pi OS build 後に代表ヘッダと manifest の存在確認 step を追加する。ヘッダ存在は install 成否のみ、由来検証は manifest 側で行う。パス基準は `<install_dir>/rootfs/usr/include/` 相対とする（`<install_dir>/rootfs/` 相対ではない点に注意）。代表は次の 3 つ。
  - `libcamera/libcamera/camera.h`（`libcamera-dev` の install 成否確認。`libcamera-dev` は upstream 慣行で `usr/include/libcamera/libcamera/` に配置されるため二重ディレクトリになる）
  - `c++/14/vector`（`libstdc++-14-dev` の install 成否確認）
  - `.webrtc-build-sysroot.json` の `deb_files` に、JSON の `packages` に列挙した 10 個の直接指定パッケージがすべて含まれること（transitive dep で `.deb` は数十件になるため件数下限だけでは意味が薄い。`deb_files` は `<pkg>_<version>_<arch>.deb` 形式のため、`<パッケージ名>_` プレフィックスで match すればよい）

### テスト

- テストは sora-python-sdk と揃えて `tests/sysroot_builder/test_sysroot_builder.py` に置く（momo 既存の E2E テスト用 `test/`（単数）とは別ディレクトリで並存させる）。`shiguredo-python` の推奨命名 `tests/test_sysroot_builder/test_*.py` からは逸脱するが、canonical (`sysroot_builder.py` 本体・そのテスト) と丸ごと揃える方を優先する。
- テスト内容は validation・fingerprint・symlink 相対化・manifest 再利用 / 拒否・`pin_priority` をネットワーク無しで検証する。
- 統合確認は `python3 run.py build raspberry-pi-os_armv8`（CI の `build-ubuntu` matrix）で行う。
- Jetson / Windows / macOS / Ubuntu x86_64 に回帰が無いこと。

## 完了条件

- `multistrap/raspberry-pi-os_armv8.conf` が削除されている。
- `sysroot_builder.py` と `sysroot/raspberry-pi-os_armv8.json`、keyring 2 件、`sysroot/keyrings/SHA256SUMS` が追加されている。
- `run.py` が Raspberry Pi OS で `install_sysroot()`、Jetson で `install_rootfs()` を呼ぶ。
- CI の Raspberry Pi OS entry から `multistrap` / insecure sed が消えている。Jetson entry には残っている。
- CI の Raspberry Pi OS entry に、`sysroot/keyrings/SHA256SUMS` を `sha256sum -c` で検証する step が入っている。
- CI の Raspberry Pi OS entry に、build 後の代表ヘッダ（`libcamera/libcamera/camera.h` と `c++/14/vector`）と `.webrtc-build-sysroot.json` の `deb_files` に直接指定パッケージ 10 個がすべて含まれることを確認する step が入っている。
- `python3 run.py build raspberry-pi-os_armv8` が成功する。
- sysroot builder のユニットテストが通る。
- `CHANGES.md` の `## develop` にエントリがある。
- 0004（Jetson）が本 issue の成果物を前提に書ける状態になっている。

## 参考

- sora-python-sdk `issues/closed/0074-change-multistrap-to-sysroot.md`
- sora-cpp-sdk `issues/closed/0002-change-replace-multistrap-with-sysroot.md`
- webrtc-build `feature/sysroot`（`sysroot_builder.py`）
- 後続: [0004-change-jetson-multistrap-to-sysroot.md](./0004-change-jetson-multistrap-to-sysroot.md)
