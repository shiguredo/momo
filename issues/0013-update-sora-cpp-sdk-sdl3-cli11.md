# sora-cpp-sdk を 2026.2.1 に同期し、SDL3 / CLI11 を最新化する

- Created: 2026-08-19
- Completed: {YYYY-MM-DD}
- Branch: feature/update-sora-cpp-sdk-sdl3-cli11
- Polished: 2026-08-19

## 目的

正式リリースに向けて、vendoring している `src/sora-cpp-sdk/` を sora-cpp-sdk の最新リリース `2026.2.1` に同期し、`SDL3` を `3.4.14`、`CLI11` を `v2.7.2` に更新する。libwebrtc は `m150.7871.3.0` のまま固定し、m152 には上げない (sora-cpp-sdk 2026.2.1 が m150 ベースのため)。

## 現状

- `DEPS` の依存バージョンは `SDL3_VERSION=3.2.24`、`CLI11_VERSION=v2.6.1`、`BOOST_VERSION=1.91.0`、`WEBRTC_BUILD_VERSION=m150.7871.3.0` で、更新が必要。`CHANGES.md` の `develop` セクションには `SDL3 3.2.24` / `CLI11 v2.6.1` の更新履歴が記録されている
- `src/sora-cpp-sdk/LAST_UPDATED` は `LAST_UPDATED_SORA_CPP_SDK=bd0af5953a972d8acf605dbbe5910a82fff9ee74` で、`2026.2.1` タグのコミットより古い。`git ls-remote` で `2026.2.1` タグのハッシュを取得して前後関係を確認する必要がある
- sora-cpp-sdk `2026.2.1` は Boost 1.92.0 を同梱していると想定されるが、sora-cpp-sdk 側の `DEPS` / `CMakeLists.txt` で要検証。momo 側の `BOOST_VERSION=1.91.0` との整合を取り、上げる場合は `BOOST_SHA256_HASH` も同時更新が必要になる
- sora-cpp-sdk `2026.2.1` が `m150.7871.3.0` ベースであることも、sora-cpp-sdk 側の `DEPS` の `WEBRTC_BUILD_VERSION` で要検証。`m152` に上がっていれば本 issue の前提が崩れるため、同期前に確認する
- `SDL3 3.4.14` は `https://github.com/libsdl-org/SDL/releases/tag/release-3.4.14`、`CLI11 v2.7.2` は `https://github.com/CLIUtils/CLI11/releases/tag/v2.7.2` で実在と最新性を要検証。`buildbase.py` の SDL3 取得 URL は `https://github.com/libsdl-org/SDL/releases/download/release-{version}/SDL3-{version}.tar.gz`、CLI11 は `git clone --branch {version} https://github.com/CLIUtils/CLI11.git` で取得可否を確認する
- sora-cpp-sdk の同期スクリプトにバグがある
  - `src/sora-cpp-sdk/README.md` の手順 `./diff-sora-cpp-sdk.sh | patch -p1` (手順 2.1) と `../momo/sora-cpp-sdk/diff-sora-cpp-sdk.sh . | patch -p1` (手順 3.1) は、いずれも `diff-sora-cpp-sdk.sh` が引数 2 個 (`<Sora C++ SDK dir>` と `<target_commit>`) 必須のため実行不能。実際のスクリプト先頭で `if [ $# -ne 2 ]` で exit する
  - `src/sora-cpp-sdk/copy-from-sora-cpp-sdk.sh` と `copy-to-sora-cpp-sdk.sh` は `find include` と `find src` のみを対象にしており、`diff-sora-cpp-sdk.sh` が `find third_party` を含むのと非対称で、`third_party` 配下の変更がコピーされない。さらに `copy-*` は既存ファイル集合を基準に `cp` するため、sora-cpp-sdk 側で新規追加・削除されたファイルは取りこぼす

## 設計方針

1. 同期前に sora-cpp-sdk `2026.2.1` タグの `WEBRTC_BUILD_VERSION` と `BOOST_VERSION` を sora-cpp-sdk 側の `DEPS` / `CMakeLists.txt` で確認する。`m150` ベースであることを確認した上で本 issue を進める。`SDL3 3.4.14` / `CLI11 v2.7.2` の実在性は各 GitHub Releases で確認する
2. `src/sora-cpp-sdk/` を同期元の `2026.2.1` タグ相当のコミットに同期する。`diff-sora-cpp-sdk.sh <sora-cpp-sdk dir> 2026.2.1` で差分を確認し、原則 `patch -p1` で反映する。`copy-from-sora-cpp-sdk.sh` を使う場合は、`third_party` 欠落修正後に加えて、新規追加・削除ファイルは `git diff --name-status` で `A` / `D` を検出して手動で反映する。同期後に `git status` で vendoring が完結していることを確認する
3. 同期スクリプトの `third_party` 欠落を修正する (`copy-from-sora-cpp-sdk.sh` / `copy-to-sora-cpp-sdk.sh` で `THIRD_PARTY_FILES=$(find third_party -type f)` を追加し、`ALL_FILES` に含める)。`diff-sora-cpp-sdk.sh` と挙動を揃える
4. `src/sora-cpp-sdk/README.md` の手順を、実際の引数 (`<Sora C++ SDK dir> <target_commit>`) に合わせて修正する。手順 2.1 は `./diff-sora-cpp-sdk.sh <sora-cpp-sdk dir> 2026.2.1 | patch -p1`、手順 3.1 は `diff-sora-cpp-sdk.sh` の 2 引数形式に修正する
5. `DEPS` の `SDL3_VERSION` を `3.4.14` に、`CLI11_VERSION` を `v2.7.2` に更新する。Boost は sora-cpp-sdk `2026.2.1` が同梱するバージョンに追従するかを、sora-cpp-sdk 側の `DEPS` と momo 側のビルド結果 (`buildbase.py` の `verify_sha256` 成否) で判断する。上げる場合は `BOOST_VERSION` と `BOOST_SHA256_HASH` を同時更新する。`BOOST_SHA256_HASH` は `curl -L https://oss-mirrors.shiguredo.jp/boost_<version_underscore>.tar.gz | sha256sum` で取得するか、sora-cpp-sdk 側のハッシュを流用する。判断結果とハッシュ取得方法は本 issue の解決方法に記録する。`CHANGES.md` の `develop` セクションに `UPDATE` エントリを追記する (shiguredo-changelog 参照)
6. libwebrtc は `m150.7871.3.0` のまま変更しない。`WEBRTC_BUILD_VERSION` は触らない
7. 実装時は sora-cpp-sdk 同期コミットと `DEPS` / `CHANGES.md` 更新コミットを分離する。1 コミット 1 論理変更とする

## 完了条件

- `src/sora-cpp-sdk/LAST_UPDATED` の `LAST_UPDATED_SORA_CPP_SDK` が `2026.2.1` タグのコミットハッシュに更新され、`LAST_UPDATED_MOMO` が同期時の momo 側 HEAD に更新されている (`update-last-updated.sh` を `2026.2.1` タグを checkout した sora-cpp-sdk で実行した出力と一致)
- `DEPS` の `SDL3_VERSION=3.4.14`、`CLI11_VERSION=v2.7.2` に更新され、`CHANGES.md` の `develop` セクションに `UPDATE` エントリが追記されている。Boost を上げる場合は `BOOST_VERSION` と `BOOST_SHA256_HASH` も正しいハッシュで更新されている
- `copy-from-sora-cpp-sdk.sh` / `copy-to-sora-cpp-sdk.sh` が `third_party` を含むよう修正され、`README.md` の手順が引数 2 個の実態に合致している
- 全プラットフォーム (`windows_x86_64` / `macos_arm64` / `ubuntu-24.04_x86_64` / `ubuntu-22.04_x86_64` / `raspberry-pi-os_armv8` / `ubuntu-22.04_armv8_jetson`) で `python3 run.py build <platform> --package` が通る。`run.py` の `AVAILABLE_TARGETS` は 7 個だが `macos_x86_64` は CI 対象外のため 6 個で判定する
- `e2e-test.yml` の E2E テスト (GitHub-hosted 3 matrix と self-hosted 4 種を含む) が `from_build: true` 経路で通る
- 下記の vendored コードに対する重要指摘が、`2026.2.1` への同期で解消されているか確認され、結果が本 issue の解決方法に表形式で記録されている。解消されていないものは既存の対応 issue がカバーしているかを確認し、カバーされていないもののみ別 issue を起票する。既存カバーは `0020` (Jetson デコーダ deadlock) / `0021` (Jetson SendEOS) / `0022` (V4L2 memcpy) / `0023` (libcamera UAF) の 4 件、残り 10 件は本 issue で初回確認する
  - VPL の `VPL_CHECK_RESULT` が負の `MFX_ERR_*` を無視 (`hwenc_vpl/vpl_utils.h` の `VPL_CHECK_RESULT` マクロ)
  - VPL の NV12 で `Data.UV` 未設定 (`hwenc_vpl/vpl_video_encoder.cpp` / `vpl_video_decoder.cpp` の `Data.UV` 設定)
  - Jetson / V4L2 の `configured_bitrate_bps_` 未初期化比較 (`hwenc_jetson/jetson_video_encoder.cpp` / `hwenc_v4l2/v4l2_h264_encoder.cpp`)
  - IVF ヘッダ除去の size アンダーフロー (`hwenc_jetson/jetson_video_encoder.cpp` / `hwenc_nvcodec/nvcodec_video_encoder.cpp` / `hwenc_vpl/vpl_video_encoder.cpp`)
  - Jetson デコーダの `CHUNK_SIZE` 超過 memcpy (`hwenc_jetson/jetson_video_decoder.cpp`)
  - Jetson エンコーダの AV1 OBU サイズ 1 バイト固定読み (`hwenc_jetson/jetson_video_encoder.cpp`)
  - NvCodec エンコーダの `frameRateNum=0` 除算ゼロ (`hwenc_nvcodec/nvcodec_video_encoder.cpp`)
  - CUDA の `cuCtxPushCurrent` が例外パスで Pop されない (`hwenc_nvcodec/nvcodec_decoder_cuda.cpp` の `cuCtxPushCurrent` / `cuCtxPopCurrent`)
  - V4L2Runner が `POLLERR` / `HUP` を無視し破棄ハング (`hwenc_v4l2/v4l2_runner.cpp`)
  - V4L2 系のデバイスパス直書き (`hwenc_v4l2/v4l2_converter.cpp`)
  - libcamerac の Span dangling (`hwenc_v4l2/libcamerac.cpp`)
  - libcamera の mmap `MAP_FAILED` 誤判定 (`hwenc_v4l2/libcamera_capturer.cpp` の `mmap` 戻り値判定)
  - NvCodec エンコーダのフレームサイズ未検証 GPU コピー (`hwenc_nvcodec/nvcodec_video_encoder.cpp`)
  - `hwenc_v4l2/v4l2_h264_decoder.cpp` の `Release()` 空実装

## 解決方法

未着手 (PR 作成後に追記する)
