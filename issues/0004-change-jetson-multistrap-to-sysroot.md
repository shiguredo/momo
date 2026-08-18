# Jetson の multistrap を署名検証付き sysroot に切り替える

- Priority: Medium
- Created: 2026-07-23
- Completed: {YYYY-MM-DD}
- Model: Composer 2.5
- Branch: feature/change-jetson-multistrap-to-sysroot
- Polished: 2026-08-18

## 目的

`ubuntu-22.04_armv8_jetson` の rootfs 生成を、残存している insecure な `multistrap` 経路から、0003 で導入した署名検証付き sysroot builder へ切り替える。これにより `multistrap/` ディレクトリと CI の insecure sed を完全に撤去する。

## 優先度根拠

Medium とする。

- 0003 完了後も Jetson 経路だけが `multistrap --no-auth` と `/usr/sbin/multistrap` の書き換えに依存したまま残る。
- NVIDIA APT と Ubuntu Ports の混在、Jetson 固有の symlink 補正があるため Raspberry Pi OS より複雑であり、0003 の後続として分離する。
- ただし現行 CI（Ubuntu 22.04）ではまだ動くため High にはしない。

## 前提

- [0003-change-raspberry-pi-os-multistrap-to-sysroot.md](./0003-change-raspberry-pi-os-multistrap-to-sysroot.md) が完了し、`sysroot_builder.py` / `sysroot/` / `install_sysroot()` が利用可能なこと。
- 0003 未完了のまま本 issue に着手しない。

## 現状

### 呼び出し経路（0003 完了後の想定）

- `run.py` の `install_deps()` が Jetson だけ `install_rootfs()`（`buildbase.py`）を呼ぶ。
- conf は `multistrap/ubuntu-22.04_armv8_jetson.conf`。
- CI の Jetson entry だけが `multistrap` install + insecure sed を実行する。

### 現行 conf

`multistrap/ubuntu-22.04_armv8_jetson.conf`:

| セクション | source | suite | packages |
|---|---|---|---|
| Ports | `http://ports.ubuntu.com` | jammy | `libc6-dev` `libstdc++-10-dev` `libxext-dev` `libxtst-dev` |
| Jetson | `https://repo.download.nvidia.com/jetson/common` | r36.3 | `nvidia-jetpack` |
| T234 | `https://repo.download.nvidia.com/jetson/t234` | r36.3 | `nvidia-l4t-camera` `nvidia-l4t-multimedia` |

- `noauth=true`。Ubuntu Ports は HTTP。NVIDIA 側に `signed_by` が無い。
- `buildbase.py` の `install_rootfs()` が Jetson 向けに `libnvbuf_fdmap.so` → `libnvbuf_fdmap.so.1.0.0` の相対 symlink を `tegra` / `nvidia` の両方で補正する。

### CMake 依存

`CMakeLists.txt` の Jetson 分岐は `CMAKE_SYSROOT` 配下の `jetson_multimedia_api` と `usr/lib/aarch64-linux-gnu/nvidia` を参照する。rootfs 配置先（`install_dir/rootfs`）は維持する。

## スコープ

含む:

1. `sysroot/ubuntu-22.04_armv8_jetson.json` を追加する。
2. NVIDIA の `jetson-ota-public.asc` と Ubuntu Ports の `ubuntu-archive-keyring.gpg` を `sysroot/keyrings/` に vendoring し、それぞれの SHA-256 を 0003 で導入した `sysroot/keyrings/SHA256SUMS` に追記する（Ubuntu 側だけ runner ローカルの `/usr/share/keyrings/` を参照する非対称な形にしない）。
3. Jetson 向け repository pin を JSON で設定する（`pin_priority` 参考値: NVIDIA `common` / `t234` = 700、Ubuntu Ports = 500。canonical (`sysroot_builder.py`) は `_PIN_PRIORITY_MIN` / `_PIN_PRIORITY_MAX` の範囲検証のみで具体値の意見を持たないため、値は先行 pending 0043 の提案表と揃える）。
4. `libnvbuf_fdmap.so` 互換 symlink 補正は `run.py` の `install_sysroot()` 直後の Jetson 専用後処理として実装する（sysroot builder 本体には手を加えず、canonical との差分を作らない）。
5. `run.py` から Jetson の `install_rootfs()` 呼び出しを削除し、`install_sysroot()` に統一する。
6. `multistrap/ubuntu-22.04_armv8_jetson.conf` と `multistrap/` ディレクトリ自体を削除する。
7. `buildbase.py` の `install_rootfs()` はリポジトリからは削除しない。呼び出し元 (run.py の Jetson 分岐) を無くすことで dead code 化するだけに留める（`buildbase.py` は melpon/buildbase テンプレートで次回 `curl -LO` 上書きで復活するため、削除しても維持できない。0003 の「テンプレート外への逸脱は次回上書きで巻き戻る」対称形）。
8. `.github/workflows/build.yml` から `multistrap` install と insecure sed を完全削除する。0003 で target 別に分割された deps step のうち、Jetson 側の `if:` 分岐と `multistrap` / sed 行を落とす。
9. 0003 で追加した `Test sysroot_builder` step の `if:` gate を Jetson matrix にも広げる（`raspberry-pi-os_armv8` に加えて `ubuntu-22.04_armv8_jetson` でも走らせる）。同 step の `python3 -m pip install --user pytest` を uv 経由に切り替える（`astral-sh/setup-uv` + `uv run --with pytest pytest tests/sysroot_builder/` など。`shiguredo-python` の「パッケージマネージャ・タスクランナーは uv のみ」に整合させる。pyproject.toml の新設はしない）。
10. `CHANGES.md` に `[CHANGE]` を追記する。

含まない:

- JetPack / L4T の同系内 suite bump（r36.3 → r36.4 等の同 JetPack 6 系内の minor 更新）。現行 conf の suite `r36.3` を起点にする（先行 pending 0043 は既に r36.4 系を選択済みだが、本 issue では現行 momo conf に忠実にする）。ただし着手時に `https://repo.download.nvidia.com/jetson/common/dists/r36.3/` が生存していることを確認し、404 なら別 issue で r36.4 系への切替判断を先に行う。r36.4 系に上げる場合は根拠と検証範囲を PR に明示する。
- Jetson 実機 E2E の新設（既存の成果物ビルド成功を完了条件の中心とする）。
- Raspberry Pi OS 側の再設計（0003 の成果を前提に触らない）。
- `buildbase.py` からの `install_rootfs()` 実装削除（上記スコープ 7 参照）。

## 設計方針

### JSON と署名

- Ubuntu Ports は HTTPS (`https://ports.ubuntu.com/ubuntu-ports`) + vendored `ubuntu-archive-keyring.gpg`（0003 の SHA256SUMS 方式に合わせて runner ローカル keyring に依存しない）。
- NVIDIA common / t234 は HTTPS + vendored `jetson-ota-public.asc`。
- `pin_priority` で NVIDIA origin (700) を Ubuntu Ports (500) より高くする（0003 で momo に取り込んだ canonical の `RepositoryConfig.pin_priority` を利用）。
- **`nvidia-jetpack` meta-package は sysroot 集合に入れず**、build に必要な個別パッケージを明示する（先行 pending 0043 の方針に倣う。meta-package は CUDA toolkit / sample / documentation 等の大規模 dependency を含み sysroot が肥大化する）。第一候補パッケージ集合:
  - `nvidia-l4t-core`
  - `nvidia-l4t-camera`
  - `nvidia-l4t-multimedia`
  - `nvidia-l4t-multimedia-utils`
  - `nvidia-l4t-jetson-multimedia-api`
  - Ubuntu Ports 側: `libc6-dev` / `libstdc++-11-dev`（JetPack 6 は Ubuntu 22.04 rootfs ベースで default toolchain は gcc-11 系のため、sysroot の C++ header も 11 系に揃える。momo 自体は `-nostdinc++` + libc++ (webrtc-build 同梱) を使うので libstdc++ に直接は依存しないが、sysroot 内の他 package が暗黙に依存する系を default に合わせる。現行 conf の `libstdc++-10-dev` も Jammy archive には存在するが default から外れた旧系のため 11 に上げる） / `libxext-dev` / `libxtst-dev`
  - sora-python-sdk pending 0043 は `libdbus-1-dev` を含み `libxtst-dev` を含まないが、momo は `CMakeLists.txt` で `Xtst` を直接 link しており `dbus` は不使用のため 1 package ずつ入れ替える。
  - `nvidia-l4t-multimedia-utils` は pending 0043 の集合に倣い暫定で含める。着手時の DT_NEEDED 検査で不要と分かれば PR で外す。
  - Sora C++ SDK の再帰 `DT_NEEDED` 検査で不足 SONAME が判明した場合は、SONAME と追加 package の対応根拠を PR に記録して package 名を明示追加する。`nvidia-jetpack` で一括補完しない。

### JSON 骨格（例）

```json
{
    "name": "ubuntu-22.04_armv8_jetson",
    "arch": "arm64",
    "triplet": "aarch64-linux-gnu",
    "packages": [
        "libc6-dev",
        "libstdc++-11-dev",
        "libxext-dev",
        "libxtst-dev",
        "nvidia-l4t-core",
        "nvidia-l4t-camera",
        "nvidia-l4t-multimedia",
        "nvidia-l4t-multimedia-utils",
        "nvidia-l4t-jetson-multimedia-api"
    ],
    "repositories": [
        {
            "url": "https://ports.ubuntu.com/ubuntu-ports",
            "suite": "jammy",
            "components": ["main", "universe"],
            "signed_by": "keyrings/ubuntu-archive-keyring.gpg",
            "pin_priority": 500
        },
        {
            "url": "https://repo.download.nvidia.com/jetson/common",
            "suite": "r36.3",
            "components": ["main"],
            "signed_by": "keyrings/jetson-ota-public.asc",
            "pin_priority": 700
        },
        {
            "url": "https://repo.download.nvidia.com/jetson/t234",
            "suite": "r36.3",
            "components": ["main"],
            "signed_by": "keyrings/jetson-ota-public.asc",
            "pin_priority": 700
        }
    ]
}
```

（実装着手時に canonical (`RepositoryConfig` の validation) のキー名と一致することを再確認する。上記は現行 conf との差分最小案。）

### NVIDIA keyring の期待値（先行 pending 0043 の調査値を参考として引き写す。着手時に一次配布物と再照合する）

- vendored `jetson-ota-public.asc` の SHA-256: `576f852981855e5c6cfb9b625ffb51b984ca451f1181b2e70435b005034fad55`
- primary fingerprint: `3C6D1FF3100C8C3ABB0869C0E6543461A9996195`
- encryption subkey fingerprint: `13ADEA72CD3B449D4C77CA7A84F3CEB8E58DF1E8`
- signing subkey fingerprint: `13804AEEB181616F3B4964270D296FFB880FB004`
- key file は primary key 1 件、encryption subkey 1 件、signing subkey 1 件だけを含むことを `gpg --show-keys --with-colons` で検証する（capability は primary が `sc`、encryption subkey が `e`、signing subkey が `s`）。

Ubuntu Ports keyring は Debian archive keyring と同様に一次配布パッケージから抽出したものを vendoring し、SHA-256 を `SHA256SUMS` に追記する。値は着手時に決定して PR に記録する。

### `libnvbuf_fdmap.so` 後処理

- 現行 `install_rootfs()` の tegra / nvidia 両対応 (`usr/lib/aarch64-linux-gnu/tegra` と `usr/lib/aarch64-linux-gnu/nvidia`) を引き継ぐ。JetPack 6 系では `nvidia/` 側が主。
- 実装はリポジトリルートに独立モジュール `jetson_postprocess.py` として置き、そこに関数 `fixup_jetson_libnvbuf_fdmap_symlinks(rootfs_dir)` を定義する（`sysroot_builder.py` canonical に手を加えない。canonical と対称に、`run.py` を丸ごと import せずに単独テストできる粒度にする）。
- `run.py` は `install_sysroot()` 直後に `jetson_postprocess.fixup_jetson_libnvbuf_fdmap_symlinks()` を呼ぶ。
- 先行 pending 0043 は canonical に `postprocess: jetson-r36` の allowlist 追加を提案しているが、canonical が pending 状態のため取り込まず、後処理は momo 側 `jetson_postprocess.py` に閉じ込める。canonical がマージされた後に置き換えるかは別 issue で判断する。
- 既存 link の誤 target / dangling / 通常ファイル上書きは拒否する（sora-python-sdk pending 0043 と同等の契約）。
- テストは 0003 で導入する `tests/sysroot_builder/` とは別に、`tests/jetson_postprocess/test_jetson_postprocess.py` を追加する（`shiguredo-python` の推奨命名 `tests/test_&lt;module&gt;/test_*.py` からは逸脱するが、0003 の `tests/sysroot_builder/` と対称形にする方を優先する）。テスト内容は temporary fixture に対する link 作成、正しい既存 link の再利用、誤 target / dangling / 通常 file の拒否とする。

### 接続

0003 完了時点の `run.py` は次のように分岐している（Jetson だけ `install_rootfs()`、Raspberry Pi OS だけ `install_sysroot()`）。

```python
if platform.target.os == "jetson":
    conf = os.path.join(BASE_DIR, "multistrap", f"{platform.target.package_name}.conf")
    ...
    install_rootfs(...)
elif platform.target.os == "raspberry-pi-os":
    config_path = os.path.join(BASE_DIR, "sysroot", f"{platform.target.package_name}.json")
    install_sysroot(config_path=config_path, install_dir=install_dir)
```

0004 完了時点で次に統一する。

`run.py` のファイル冒頭で `from jetson_postprocess import fixup_jetson_libnvbuf_fdmap_symlinks` を追加し、`install_deps()` の中で次のように接続する。

```python
if platform.target.os in ("jetson", "raspberry-pi-os"):
    config_path = os.path.join(BASE_DIR, "sysroot", f"{platform.target.package_name}.json")
    install_sysroot(config_path=config_path, install_dir=install_dir)
    if platform.target.os == "jetson":
        fixup_jetson_libnvbuf_fdmap_symlinks(os.path.join(install_dir, "rootfs"))
```

- `run.py` の `install_rootfs` import (0003 完了時点で `buildbase` から import されている) は呼び出し元が消えるため 0004 で外す (`install_rootfs` 実装は `buildbase.py` に残す)。
- `run.py` の `hashlib` import も、`install_rootfs` の呼び出しで conf の MD5 を計算するためだけに存在するので、0004 で外す (ruff F401 で拾える dead import)。
- Jetson 専用の `rootfs.version` / MD5 キャッシュは作らない。
- ローカルに旧 multistrap 由来の `_install/ubuntu-22.04_armv8_jetson/*/rootfs` がある場合は `rootfs` ディレクトリの手動削除が必要である旨を `CHANGES.md` に書く（`rootfs.version` は新経路では読まれないが、混乱を避けるため合わせて削除するのが望ましい）。

### CI

- Jetson entry から `multistrap` / insecure sed を削除する。0003 で target 別に分割された deps step のうち、Jetson 用の `multistrap` install と sed step を落とす。
- `binutils-aarch64-linux-gnu` は残す。
- vendored NVIDIA keyring + Ubuntu Ports keyring の SHA-256 検証は 0003 で導入した `sysroot/keyrings/SHA256SUMS` に期待値を追記し、`sha256sum -c` で検証する（0003 の CI step を Jetson entry にも通す）。
- Jetson OS build 後の代表パス存在確認 step を追加する。パス基準は `<install_dir>/rootfs/` 相対とする。代表は次の 4 つ。
  - `usr/lib/aarch64-linux-gnu/nvidia/libnvbuf_fdmap.so.1.0.0`（NVIDIA ライブラリ実体）
  - `usr/lib/aarch64-linux-gnu/nvidia/libnvbuf_fdmap.so`（後処理で作成した相対 symlink）
  - `usr/src/jetson_multimedia_api/include/NvBufSurface.h`（Jetson Multimedia API ヘッダ）
  - `.webrtc-build-sysroot.json` の `deb_files` に、JSON の `packages` に列挙した直接指定パッケージがすべて含まれること（`<パッケージ名>_` プレフィックス match）

## 完了条件

- `multistrap/` がリポジトリから無くなっている。
- CI 全体から `multistrap` パッケージ install と `/usr/sbin/multistrap` の書き換えが消えている。
- `run.py` の rootfs 生成が `install_sysroot()` のみになっている（Jetson の `install_rootfs()` 呼び出しが消えている。`buildbase.py` の `install_rootfs()` 実装は melpon/buildbase テンプレート由来のためリポジトリからは削除せず、呼び出し元だけを無くす）。
- `sysroot/ubuntu-22.04_armv8_jetson.json` が追加され、Ubuntu Ports = 500 / NVIDIA common / t234 = 700 の `pin_priority` を持ち、`nvidia-jetpack` meta-package が入っていない。
- `sysroot/keyrings/jetson-ota-public.asc` と `sysroot/keyrings/ubuntu-archive-keyring.gpg` が追加され、`sysroot/keyrings/SHA256SUMS` にそれぞれの期待 SHA-256 が追記されている。
- CI の Jetson entry に、`sysroot/keyrings/SHA256SUMS` を `sha256sum -c` で検証する step が入っている（0003 の CI step を Jetson entry にも通す）。
- CI の Jetson entry に、build 後の代表パス（`usr/lib/aarch64-linux-gnu/nvidia/libnvbuf_fdmap.so.1.0.0`、`usr/lib/aarch64-linux-gnu/nvidia/libnvbuf_fdmap.so` symlink、`usr/src/jetson_multimedia_api/include/NvBufSurface.h`）と `.webrtc-build-sysroot.json` の `deb_files` に直接指定パッケージがすべて含まれることを確認する step が入っている。
- リポジトリルートに `jetson_postprocess.py` が追加され、`run.py` の `install_sysroot()` 直後に Jetson 専用後処理 (`libnvbuf_fdmap.so` の tegra / nvidia 両対応 symlink 補正) が呼ばれ、そのユニットテストが `tests/jetson_postprocess/` に追加され通っている。
- `run.py` から `install_rootfs` と `hashlib` の import が消えている (`buildbase.py` の `install_rootfs` 実装は残る)。
- `python3 run.py build ubuntu-22.04_armv8_jetson` が成功する。
- Raspberry Pi OS / 他プラットフォームに回帰が無い。
- `CHANGES.md` の `## develop` にエントリがある。

## 参考

- 先行: [0003-change-raspberry-pi-os-multistrap-to-sysroot.md](./0003-change-raspberry-pi-os-multistrap-to-sysroot.md)
- sora-python-sdk `issues/pending/0043-change-jetson-platform.md`（Jetson sysroot / NVIDIA keyring / `libnvbuf_fdmap.so` / meta-package 回避の議論。**pending 状態のため方針は保留中**であり、参考値・判断材料として引用する扱いにとどめる）
- 現行 `multistrap/ubuntu-22.04_armv8_jetson.conf`
- 現行 `buildbase.py` の `install_rootfs()` 内 Jetson symlink 補正
