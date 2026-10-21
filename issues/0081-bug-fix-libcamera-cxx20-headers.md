# Raspberry Pi OS のビルドが libcamera の C++20 ヘッダーで失敗する

- Created: 2026-10-21
- Completed: {YYYY-MM-DD}
- Branch: feature/fix-libcamera-cxx20-headers
- Polished: {YYYY-MM-DD}

## 目的

Raspberry Pi OS 向けの GitHub Actions ビルド (`raspberry-pi-os_armv8`) が 2026-10-16 以降、毎日失敗している。sysroot に取り込む libcamera のヘッダーが C++20 必須になった一方、momo 側の `camerac` ターゲットが C++17 でコンパイルされるため、`std::span` が見つからずに失敗する。Raspberry Pi OS 版の momo を継続してビルドできるよう修正する。

## 現状

- GitHub Actions の `build-workflow` (スケジュール実行) で、`Build momo for raspberry-pi-os_armv8` が 2026-10-16 / 10-19 / 10-20 / 10-21 に失敗している。直近の成功は 2026-10-15
- 失敗するのは `CMakeFiles/camerac.dir/src/sora-cpp-sdk/third_party/libcamerac/libcamerac.cpp.o` のコンパイルで、`rootfs/usr/include/libcamera/libcamera/base/span.h` と `controls.h` を読んだ時点で次のエラーになる

  ```
  span.h:16:48: error: no member named 'dynamic_extent' in namespace 'std'
  controls.h:52:21: error: no member named 'span' in namespace 'std'
  ```

- sysroot は `sysroot_builder.py` が `sysroot/raspberry-pi-os_armv8.json` の設定に従って APT から取得する。`libcamera-dev` はバージョンを固定しておらず、`archive.raspberrypi.com` の trixie (`pin_priority` 990) から最新版が入る
- Raspberry Pi アーカイブで `libcamera-dev` を比較すると、`0.7.2+rpt20260817-1` の `libcamera/base/span.h` は「C++20 の `std::span` を C++11 で使えるようにする自作実装」で、C++17 でもコンパイルできた。2026-10-15 に投入された `0.7.2+rpt20261009-1` では `span.h` が `#include <span>` と `using Span = std::span` の別名に置き換わり、`controls.h` も `std::span` を直接使うようになった。C++20 未満では `<span>` が空になるため上記のエラーになる
- momo 側は `CMakeLists.txt` の `camerac` ターゲットで `set_target_properties(camerac PROPERTIES CXX_STANDARD 17 C_STANDARD 99)` と C++17 を指定している。本体 `momo` ターゲットは `CXX_STANDARD 20` である
- `src/sora-cpp-sdk/third_party/libcamerac/libcamerac.cpp` は `libcamera/base/span.h` を include し、配列コントロールの設定で `libcamera::Span<const T>` を 4 箇所で構築している
- Debian 本家 trixie の `libcamera-dev` は `0.4.0-7` で、古い `libcamera::Span` の宣言は維持されている (アーカイブ上の deb を展開して確認済み)
- 本件は momo のコード変更ではなく、sysroot に流れ込む上流パッケージの更新によってビルド構成が合わなくなったものである。momo のバグではない

## 設計方針

- `camerac` ターゲットの `CXX_STANDARD` を 20 に上げ、本体 `momo` ターゲットと同じ標準に揃える
- `libcamerac.cpp` は `libcamera::Span` の別名を使い続ける。新しい libcamera では `std::span` の別名 (deprecated 付き)、古い libcamera では自作実装の別名を指すため、新旧どちらのヘッダーでもコンパイルできる。`std::span` を直接書くと旧 libcamera 環境で壊れるため、直接参照には変更しない
- 現状のビルド設定には警告をエラーにする指定が無いため、新しい libcamera で `libcamera::Span` の別名に付く deprecated 警告はビルドを止めない。この前提は修正時に再確認する
- 修正は momo の `CMakeLists.txt` のみで完結させる。sysroot の `libcamera-dev` バージョン固定は行わない (Raspberry Pi OS の更新に追従できなくなるため)

## 完了条件

- Raspberry Pi OS 向けの CI ビルドが成功する
- `camerac` が C++20 でコンパイルされ、`libcamerac.cpp` が C++17 前提の記述を含まない
- 他のプラットフォームのビルドに影響しない
