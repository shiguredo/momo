# Raspberry Pi OS のビルドが libcamera の C++20 ヘッダーで失敗する

- Created: 2026-10-21
- Completed: 2026-10-21
- Branch: feature/fix-libcamera-cxx20-headers
- Polished: 2026-10-21

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
- Raspberry Pi アーカイブで `libcamera-dev` の deb を展開して比較すると、`0.7.2+rpt20260817-1` の `libcamera/base/span.h` は「C++20 の `std::span` を C++11 で使えるようにする自作実装」で、`libcamera::Span` はクラステンプレート本体として定義されていた。この版は C++17 でもコンパイルできた。`0.7.2+rpt20261009-1` では `span.h` が `#include <span>` と `using Span = std::span` の別名に置き換わり、`controls.h` も `std::span` を直接使うようになった。C++20 未満では `<span>` が空になるため上記のエラーになる
- 新しいヘッダーでは `camera.h` や `framebuffer.h` が `libcamera/base/span.h` の include を持たないため、`libcamerac.cpp` が明示している `#include <libcamera/base/span.h>` を残さないと `libcamera::Span` 自体が解決しない
- momo 側は `CMakeLists.txt` の `camerac` ターゲットで `set_target_properties(camerac PROPERTIES CXX_STANDARD 17 C_STANDARD 99)` と C++17 を指定している。本体 `momo` ターゲットは `CXX_STANDARD 20` である。`camerac` に対する `CXX_STANDARD` の指定はこの 1 箇所だけで、`camerac` 向けの `target_compile_options` や `CMAKE_CXX_FLAGS` の上書きも無い
- `camerac` は `CMakeLists.txt` の `if (USE_V4L2_ENCODER)` 内でのみ定義され、`run.py` が `raspberry-pi-os` のときだけ `-DUSE_V4L2_ENCODER=ON` を渡す。したがって他プラットフォームのビルドには関与しない
- `src/sora-cpp-sdk/third_party/libcamerac/libcamerac.cpp` は `libcamera/base/span.h` を include し、配列コントロールの設定で `libcamera::Span<const T>` を 4 箇所で構築している
- `libcamerac.cpp` の `libcamerac_Camera_generateConfiguration` は `generateConfiguration` に `std::vector<libcamera::StreamRole>` を渡しており、新しいヘッダーで引数が `std::span` になっても `std::vector` からの変換で成立する
- `libcamera/base/span.h` 以外に、新旧のヘッダーで変わった API のうち momo が参照しているものは、この span まわり以外に無い。`property_ids.h` の改名 (`PIXEL_ARRAY_AREA` など) の対象プロパティは `src/sora-cpp-sdk/src/hwenc_v4l2/libcamera_capturer.cpp` でコメントアウトされており使われていない
- 本件は momo のコード変更ではなく、momo 側の C++17 固定と sysroot に流れ込む上流ヘッダーの C++20 要求が食い違ったものである。momo のバグではない

## 設計方針

- `camerac` ターゲットの `CXX_STANDARD` を 20 に上げ、本体 `momo` ターゲットと同じ標準に揃える。変更は `CMakeLists.txt` の `camerac` の `CXX_STANDARD` の 1 箇所に閉じる
- `libcamerac.cpp` は無変更で維持する。`libcamera::Span` と `#include <libcamera/base/span.h>` はそのまま残す。旧ヘッダーでは自作実装クラス、新ヘッダーでは `std::span` の別名 (deprecated 付き) を指すため、`libcamera::Span` を書くコードは新旧どちらのヘッダーでも通る。`std::span` を直接書くと旧ヘッダーで別型になり通らないため、直接参照には変更しない
- `libcamerac.cpp` は `src/sora-cpp-sdk/copy-to-sora-cpp-sdk.sh` により sora-cpp-sdk 側の元ファイルになる。無変更で済むため sora-cpp-sdk 側の変更も `LAST_UPDATED` の同期も不要であり、この点を完了条件で確認する。C++ 標準は `camerac` を定義する momo の `CMakeLists.txt` だけが決める
- 現状のビルド設定には警告をエラーにする指定が無い (`CMakeLists.txt`、`run.py`、`.github/workflows/build.yml` に `-Werror` 相当は無く、`camerac` に `target_compile_options` も無い)。新しいヘッダーで `libcamera::Span` の別名に付く deprecated 警告はビルドを止めない。この前提は、CI の Raspberry Pi OS 向けビルドが deprecated 警告のみで成功することで確認する
- sysroot の `libcamera-dev` バージョン固定は行わない (Raspberry Pi OS の更新に追従できなくなるため)

## 完了条件

- Raspberry Pi OS 向けの CI ビルド (`Build momo for raspberry-pi-os_armv8`) が成功する。新しいヘッダーで `libcamera::Span` の deprecated 警告は出てよいが、ビルドは失敗しない
- `camerac` の C++ 標準が 20 でコンパイルされる
- `libcamerac.cpp` が無変更 (`#include <libcamera/base/span.h>` と `libcamera::Span` の 4 箇所が維持され、`std::span` を直接参照していない) のままで、古い `libcamera-dev` (`0.7.2+rpt20260817-1`) のヘッダーに対しても C++20 でコンパイルできる
- 差分が `CMakeLists.txt` の `camerac` の `CXX_STANDARD` に限定され、sora-cpp-sdk 側のファイルと `LAST_UPDATED` に差分が出ていない
- `camerac` は Raspberry Pi OS ビルド専用のため他プラットフォームのビルドに影響しない。CI の他のビルドジョブが従来どおり成功する

## 解決方法

2026-10-21 に解決した。

- `CMakeLists.txt` の `if (USE_V4L2_ENCODER)` 内で `set_target_properties(camerac PROPERTIES CXX_STANDARD 20 C_STANDARD 99)` に変更し、`camerac` を本体 `momo` と同じ C++20 でコンパイルするようにした。理由をコメントとして残した
- `libcamerac.cpp` は無変更で維持した。`#include <libcamera/base/span.h>` と `libcamera::Span` の 4 箇所はそのままで、新旧どちらのヘッダーでも通る
- Raspberry Pi アーカイブの deb を展開したヘッダーで `libcamerac.cpp` を構文チェックした。`0.7.2+rpt20261009-1` と C++20 は deprecated 警告 4 件のみで成功、`0.7.2+rpt20260817-1` と C++20 は警告なしで成功、`0.7.2+rpt20261009-1` と C++17 は修正前のエラーを再現した
- 差分は `CMakeLists.txt` の 2 行のみで、sora-cpp-sdk 側のファイルと `LAST_UPDATED` に差分は無い
- 検証: `feature/fix-libcamera-cxx20-headers` の CI で `Build momo for raspberry-pi-os_armv8` を含む全ビルドジョブと E2E テストが成功した
- `CHANGES.md` の `## develop` に FIX のエントリを追記した
