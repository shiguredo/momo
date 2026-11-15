# Raspberry Pi OS 向けビルドを libcamera 0.8.0 に追従する

- Created: 2026-11-15
- Completed: {YYYY-MM-DD}
- Branch: feature/update-support-libcamera-0.8
- Polished: {YYYY-MM-DD}

## 目的

Raspberry Pi の apt リポジトリに libcamera 0.8.0 が配布されたため、momo の Raspberry Pi OS 向けビルドを 0.8.0 に追従させる。

libcamera は soname で ABI が分かれており、0.8 系でビルドした `libcamerac.so` は `libcamera.so.0.8` に依存する。0.7 系しか入っていない端末では読み込めないため、追従を遅らせると、Raspberry Pi OS を更新した利用者との組み合わせで libcamera を利用したカメラキャプチャが動かなくなる期間が長くなる。

## 現状

- `sysroot/raspberry-pi-os_armv8.json` は `libcamera-dev` のバージョンを固定しておらず、`archive.raspberrypi.com` の trixie (pin priority 990) から最新版を取得する
- 2026-11-15 に `sysroot_builder.py` で生成した sysroot では `libcamera-dev` / `libcamera0.8` / `libcamera-ipa` が 0.8.0+rpt20261104-1 になり、`.webrtc-build-sysroot.json` の `deb_files` に `libcamera-dev_0.8.0+rpt20261104-1_arm64.deb` と `libcamera0.8_0.8.0+rpt20261104-1_arm64.deb` が記録される。`libpisp1` は `libcamera0.8` の依存として入る
- libcamera 0.7.2+rpt20261009-1 から 0.8.0+rpt20261104-1 のパブリック API 差分は、`orientationFromRotation()` / `transformFromRotation()` の `std::optional` 返却化と、コントロール enum 名の接頭辞削除 (`MeteringSpot` から `Spot`、`AwbAuto` から `Auto` など) である。enum の数値は変わらない
- momo の `src/sora-cpp-sdk/third_party/libcamerac/libcamerac.cpp` は libcamera の enum シンボルを参照していない。`parse_enum_control()` の独自テーブルは 0.8 の接頭辞なしの名前を元から使っている
- `src/sora-cpp-sdk/src/hwenc_v4l2/libcamera_capturer.cpp` も、0.8 で変更された API を参照していない
- `libcamerac.cpp` / `libcamerac.h` / `libcameracpp.h` は sora-cpp-sdk の最新版と一致しており、sora-cpp-sdk は既に libcamera 0.8.0 への追従を完了している
- `CMakeLists.txt` の `camerac` ターゲットは、libcamera 0.7.2+rpt20261009-1 以降の `std::span` 要求に合わせて C++20 でビルドしている。0.8.0 でもこの設定のままでビルドできる
- 公開済みの momo 2026.1.1 の Raspberry Pi OS 向け `libcamerac.so` は `libcamera.so.0.7` と `libcamera-base.so.0.7` に依存する
- `doc/SETUP_RASPBERRY_PI.md` は 2025.1.3 を例に `libcamera0.7` のインストールを案内しており、libcamera 0.8 系でビルドしたバイナリ向けの案内がない

## 設計方針

- ソースコードは変更しない。0.8.0 の API 差分は momo が利用していないため、`libcamerac` と `CMakeLists.txt` は現状のままで成立する
- 0.8.0 の sysroot で Raspberry Pi OS 向けビルドが成功することを確認する
- sysroot は設定が変わらない限り再利用されるため、既存の 0.7 系 sysroot が残っているローカル環境では `_install/raspberry-pi-os_armv8/release/rootfs` を削除して再生成する。`run.py` の `install_sysroot()` は force を露出しない方針のため、CLI は変更しない
- `doc/SETUP_RASPBERRY_PI.md` の例を最新リリースの 2026.1.1 に更新し、libcamera 0.8 系でビルドしたバイナリ向けに `libcamera0.8` のインストール手順を追加する。SONAME 固定のパッケージ名を唯一の手順にしない既存方針は維持する
- `CHANGES.md` の `## develop` に `[UPDATE]` で追従内容と sysroot 再生成の注意を記載する
- Raspberry Pi 実機での E2E は CI の self-hosted Raspberry Pi runner で確認する

## 完了条件

- `python3 sysroot_builder.py --config sysroot/raspberry-pi-os_armv8.json --dest _install/raspberry-pi-os_armv8/release/rootfs` で 0.8.0+rpt20261104-1 の sysroot が生成される
- `python3 run.py build raspberry-pi-os_armv8` が成功する
- `_build/raspberry-pi-os_armv8/release/momo/libcamerac.so` の `NEEDED` に `libcamera.so.0.8` と `libcamera-base.so.0.8` が含まれる
- `doc/SETUP_RASPBERRY_PI.md` が libcamera 0.8 系のバイナリと `libcamera0.8` の対応に触れている
- `CHANGES.md` に追従内容が記載されている
- Raspberry Pi 実機で libcamera を利用した送受信が動作する (CI の self-hosted runner で確認する)
