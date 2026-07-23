# Jetson の multistrap を署名検証付き sysroot に切り替える

- Priority: Medium
- Created: 2026-07-23
- Completed: {YYYY-MM-DD}
- Model: Composer 2.5
- Branch: feature/change-jetson-multistrap-to-sysroot
- Polished: {YYYY-MM-DD}

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
2. NVIDIA の `jetson-ota-public.asc` を `sysroot/keyrings/` に vendoring し、SHA-256 を記録する。
3. Jetson 向け repository pin（NVIDIA origin を Ubuntu より優先）を JSON で設定する。
4. `libnvbuf_fdmap.so` 互換 symlink を sysroot builder の後処理、または `install_sysroot()` 直後の Jetson 専用後処理として移す。
5. `run.py` から Jetson の `install_rootfs()` 呼び出しを削除し、`install_sysroot()` に統一する。
6. `multistrap/ubuntu-22.04_armv8_jetson.conf` と `multistrap/` ディレクトリ自体を削除する。
7. `buildbase.py` の `install_rootfs()` を削除する（呼び出し元が無くなるため）。
8. `.github/workflows/build.yml` から `multistrap` install と insecure sed を完全削除する。
9. `CHANGES.md` に `[CHANGE]` を追記する。

含まない:

- JetPack / L4T のメジャー更新（r36.3 → r36.4 等）。現行 conf の suite を起点にし、上げる場合は根拠と検証範囲を PR に明示する。上げない選択も可。
- Jetson 実機 E2E の新設（既存の成果物ビルド成功を完了条件の中心とする）。
- Raspberry Pi OS 側の再設計（0003 の成果を前提に触らない）。

## 設計方針

### JSON と署名

- Ubuntu Ports は HTTPS（`https://ports.ubuntu.com/ubuntu-ports`）+ `/usr/share/keyrings/ubuntu-archive-keyring.gpg`（または同等の検証済み経路）。
- NVIDIA common / t234 は HTTPS + vendored `jetson-ota-public.asc`。
- `pin_priority` で NVIDIA origin を Ubuntu より高くする（0003 の `pin_priority` 機構を再利用）。
- `nvidia-jetpack` meta-package をそのまま入れるか、ビルドに必要な個別パッケージへ展開するかは実装時に決める。
  - 維持する場合: 現行 conf との差分を最小化できるが、sysroot が肥大化する。
  - 展開する場合: sora-python-sdk 0043 の方針（meta を避け必要パッケージを明示）を参考にし、不足 SONAME はビルド失敗から逆算して追加する。
  - いずれを採っても PR 本文に根拠を書く。

### `libnvbuf_fdmap.so` 後処理

- 現行 `install_rootfs()` の tegra / nvidia 両対応を引き継ぐ。
- 場当たり的なターゲット名分岐ではなく、JSON の allowlist 済み `postprocess`（例: `jetson-r36`）で選択する案を第一候補とする（sora-python-sdk 0043 と同様）。
- `postprocess` を fingerprint に含め、生成形式が変わる場合は `MANIFEST_VERSION` を更新する。
- 既存 link の誤 target / dangling / 通常ファイル上書きは拒否する。

### 接続

```python
if platform.target.os in ("jetson", "raspberry-pi-os"):
    config_path = os.path.join(BASE_DIR, "sysroot", f"{platform.target.package_name}.json")
    install_sysroot(config_path=config_path, install_dir=install_dir)
```

- Jetson 専用の `rootfs.version` / MD5 キャッシュは作らない。
- ローカルに旧 multistrap 由来の `_install/ubuntu-22.04_armv8_jetson/*/rootfs` がある場合は手動削除が必要である旨を `CHANGES.md` に書く。

### CI

- Jetson entry から `multistrap` / insecure sed を削除する。
- `binutils-aarch64-linux-gnu` は残す。
- vendored NVIDIA keyring の SHA-256 検証と、sysroot 内の代表パス（`jetson_multimedia_api`、`nvidia` 配下ライブラリ、`libnvbuf_fdmap.so` link）の存在確認 step を追加する。

## 完了条件

- `multistrap/` がリポジトリから無くなっている。
- CI 全体から `multistrap` パッケージ install と `/usr/sbin/multistrap` の書き換えが消えている。
- `buildbase.py` に `install_rootfs()` が残っていない。
- `run.py` の rootfs 生成が `install_sysroot()` のみになっている。
- `python3 run.py build ubuntu-22.04_armv8_jetson` が成功する。
- Raspberry Pi OS / 他プラットフォームに回帰が無い。
- `CHANGES.md` の `## develop` にエントリがある。

## 参考

- 先行: [0003-change-raspberry-pi-os-multistrap-to-sysroot.md](./0003-change-raspberry-pi-os-multistrap-to-sysroot.md)
- sora-python-sdk `issues/0043-change-jetson-platform.md`（Jetson sysroot / NVIDIA keyring / `libnvbuf_fdmap.so` / meta-package 回避の議論）
- 現行 `multistrap/ubuntu-22.04_armv8_jetson.conf`
- 現行 `buildbase.py` の `install_rootfs()` 内 Jetson symlink 補正
