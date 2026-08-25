# sora-cpp-sdk を 2026.2.1 に同期し、SDL3 / CLI11 を最新化する

- Created: 2026-08-19
- Completed: {YYYY-MM-DD}
- Branch: feature/update-sora-cpp-sdk-sdl3-cli11
- Polished: 2026-08-20
- Milestone: 2026.1.0

## 目的

正式リリースに向けて、vendoring している `src/sora-cpp-sdk/` を sora-cpp-sdk の最新リリース `2026.2.1` に同期し、`SDL3` を `3.4.14`、`CLI11` を `v2.7.2` に更新する。libwebrtc は m152 には上げず m150 系のままとする。momo の `WEBRTC_BUILD_VERSION` は `m150.7871.3.0` のまま維持することを基本とするが、sora-cpp-sdk 2026.2.1 は `m150.7871.3.1` ベースのため、同期後のビルドで libwebrtc API の不一致によりビルドできない場合は `m150.7871.3.1` への追従を検討する。

## 現状

- `DEPS` の依存バージョンは `SDL3_VERSION=3.2.24`、`CLI11_VERSION=v2.6.1`、`BOOST_VERSION=1.91.0`、`WEBRTC_BUILD_VERSION=m150.7871.3.0`。`SDL3` と `CLI11` は更新が必要で、`CHANGES.md` の `develop` セクションには `SDL3 3.2.24` / `CLI11 v2.6.1` の更新履歴が記録されている。Boost は sora-cpp-sdk `2026.2.1` に合わせて `1.92.0` に上げ (設計方針 5)、libwebrtc は m150 系のまま維持する (ビルド不整合時のみ `m150.7871.3.1` への追従を検討。設計方針 6)
- `src/sora-cpp-sdk/LAST_UPDATED` は `LAST_UPDATED_SORA_CPP_SDK=bd0af5953a972d8acf605dbbe5910a82fff9ee74` (2026-06-05 の canary コミット) で、`2026.2.1` タグのコミット `1be196d647797d19c02d56bb054a3de6d81edd0a` (2026-08-18) より古い。同期前に `git fetch origin 2026.2.1` でタグを取得し、`git merge-base --is-ancestor bd0af5953a972d8acf605dbbe5910a82fff9ee74 2026.2.1` で祖先関係を確認すること (`git ls-remote` はハッシュの取得のみで前後関係は判定できない)
- sora-cpp-sdk `2026.2.1` の `DEPS` は `BOOST_VERSION=1.92.0` / `BOOST_SHA256_HASH=c4a3b310ddd2472416e091067166b0713be97c63f38c212c484ada022fd296ce`。momo 側の `BOOST_VERSION=1.91.0` との整合を取り、上げる場合は `BOOST_VERSION` と `BOOST_SHA256_HASH` を同時更新する
- sora-cpp-sdk `2026.2.1` の `DEPS` の `WEBRTC_BUILD_VERSION` は `m150.7871.3.1` で、m150 系 (m152 ではない) であることは確認済み。ただし momo の `m150.7871.3.0` とはパッチ差分があるため、同期後のビルドで libwebrtc API の不一致が出ないか確認が必要
- sora-cpp-sdk `2026.2.1` の `DEPS` は `VPL_VERSION=v2.17.0` / `CMAKE_VERSION=4.4.2` / `CUDA_VERSION=13.3.1-1` で、momo の `VPL_VERSION=v2.16.0` / `CMAKE_VERSION=4.3.2` / `CUDA_VERSION=12.9.1-1` と乖離している。momo の VPL / CMAKE は libwebrtc (m150.7871.3.0) の webrtc-build に合わせた値、CUDA は `CHANGES.md` で独立に更新された値であり、本 issue では変更しないことを基本とする (設計方針 6)
- `SDL3 3.4.14` と `CLI11 v2.7.2` は GitHub Releases で実在と最新性を確認済み (2026-08-19 時点)。`buildbase.py` の SDL3 取得 URL は `https://github.com/libsdl-org/SDL/releases/download/release-{version}/SDL3-{version}.tar.gz`、CLI11 は `git clone --branch {version} https://github.com/CLIUtils/CLI11.git` で取得する
- sora-cpp-sdk の同期スクリプトにバグがある
  - `src/sora-cpp-sdk/README.md` の手順 `./diff-sora-cpp-sdk.sh | patch -p1` (手順 2.1) と `../momo/sora-cpp-sdk/diff-sora-cpp-sdk.sh . | patch -p1` (手順 3.1) は、いずれも `diff-sora-cpp-sdk.sh` が引数 2 個 (`<Sora C++ SDK dir>` と `<target_commit>`) 必須のため実行不能。実際のスクリプト先頭で `if [ $# -ne 2 ]` で exit する
  - `src/sora-cpp-sdk/copy-from-sora-cpp-sdk.sh` と `copy-to-sora-cpp-sdk.sh` は `find include` と `find src` のみを対象にしており、`diff-sora-cpp-sdk.sh` が `find third_party` を含むのと非対称で、`third_party` 配下の変更がコピーされない。さらに `copy-*` は既存ファイル集合を基準に `cp` するため、sora-cpp-sdk 側で新規追加・削除されたファイルは取りこぼす

## 設計方針

1. sora-cpp-sdk `2026.2.1` タグの `DEPS` で `WEBRTC_BUILD_VERSION` (`m150.7871.3.1`) と `BOOST_VERSION` (`1.92.0`) を確認済み。m150 系であることと `SDL3 3.4.14` / `CLI11 v2.7.2` の実在 (2026-08-19 時点) を確認した上で本 issue を進める。実装時点でも最新リリースを再確認すること
2. `src/sora-cpp-sdk/` を同期元の `2026.2.1` タグ相当のコミットに同期する。`diff-sora-cpp-sdk.sh <sora-cpp-sdk dir> 2026.2.1` で差分を確認し、原則 `patch -p1` で反映する。`diff-sora-cpp-sdk.sh` は momo 既存ファイル集合のみを対象とするため、新規追加・削除ファイルは `git diff --name-status <LAST_UPDATED_SORA_CPP_SDK>..2026.2.1 -- include src third_party` で `A` / `D` を検出して手動で反映する (新規追加ファイルは `cp`、削除ファイルは `git rm` で反映する。`copy-from-sora-cpp-sdk.sh` を使う場合も同様)。同期後に `git status` で vendoring が完結していることを確認する
3. 同期スクリプトの `third_party` 欠落を修正する (`copy-from-sora-cpp-sdk.sh` / `copy-to-sora-cpp-sdk.sh` で `THIRD_PARTY_FILES=$(find third_party -type f)` を追加し、`ALL_FILES` に含める)。`diff-sora-cpp-sdk.sh` と挙動を揃える。修正後も `copy-*` は既存ファイル集合基準の `cp` のため、新規追加・削除ファイルの反映は設計方針 2 の手動対応が前提となる。同期を実行可能かつ完結させるための前提修正であり、本 issue に含める
4. `src/sora-cpp-sdk/README.md` の手順を、実際の引数 (`<Sora C++ SDK dir> <target_commit>`) に合わせて修正する。手順 2.1 は `./diff-sora-cpp-sdk.sh <sora-cpp-sdk dir> <target_commit> | patch -p1`、手順 3.1 は `../momo/sora-cpp-sdk/diff-sora-cpp-sdk.sh <sora-cpp-sdk dir> <target_commit> | patch -p1` の 2 引数形式に修正する。同期手順を実行可能にするための前提修正であり、本 issue に含める。同期スクリプト修正は機能に直接影響しない変更のため `CHANGES.md` の `### misc` サブセクションに記録し、`README.md` 手順修正はドキュメント変更のため `CHANGES.md` には記録しない (shiguredo-changelog 参照)
5. `DEPS` の `SDL3_VERSION` を `3.4.14` に、`CLI11_VERSION` を `v2.7.2` に更新する。Boost は sora-cpp-sdk `2026.2.1` が同梱する `1.92.0` に上げ、`BOOST_VERSION` と `BOOST_SHA256_HASH` を同時更新する。`BOOST_SHA256_HASH` は `curl -L https://oss-mirrors.shiguredo.jp/boost_1_92_0.tar.gz | sha256sum` で取得した値と、sora-cpp-sdk 側の `DEPS` の値 (`c4a3b310ddd2472416e091067166b0713be97c63f38c212c484ada022fd296ce`) を照合して確定する。判断結果とハッシュ取得方法は本 issue の解決方法に記録する。`CHANGES.md` の `develop` セクションに `UPDATE` エントリを追記する (shiguredo-changelog 参照)。エントリ分割粒度は既存の慣行 (libwebrtc m150 更新で関連依存を 1 エントリにまとめる) に合わせ、sora-cpp-sdk 2026.2.1 同期と依存更新 (SDL3 / CLI11 / Boost) を 1 エントリにまとめる
6. libwebrtc は m152 には上げない。momo の `WEBRTC_BUILD_VERSION` は `m150.7871.3.0` のまま維持することを基本とする。sora-cpp-sdk 2026.2.1 は `m150.7871.3.1` ベースのため、同期後のビルドで libwebrtc API の不一致によりビルドが通らない場合は `m150.7871.3.1` への追従を検討し、判断結果を本 issue の解決方法に記録する。VPL / CMAKE / CUDA も momo の現行値 (`VPL_VERSION=v2.16.0` / `CMAKE_VERSION=4.3.2` / `CUDA_VERSION=12.9.1-1`) を維持することを基本とし、同期後のビルドで API 不一致によりビルドが通らない場合は sora-cpp-sdk `2026.2.1` の DEPS 値への追従を検討し、判断結果を本 issue の解決方法に記録する
7. 実装時は次の順序で進め、各論理変更を 1 コミットに分離する (1 コミット 1 論理変更): 同期スクリプト修正 (設計方針 3) → `README.md` 手順修正 (設計方針 4) → sora-cpp-sdk 同期 (設計方針 2) → `DEPS` / `CHANGES.md` 更新 (設計方針 5)

## 完了条件

- `src/sora-cpp-sdk/LAST_UPDATED` の `LAST_UPDATED_SORA_CPP_SDK` が `2026.2.1` タグのコミットハッシュに更新され、`LAST_UPDATED_MOMO` が同期時の momo 側 HEAD に更新されている (`update-last-updated.sh` を `2026.2.1` タグを checkout した sora-cpp-sdk で実行した出力と一致)
- `DEPS` の `SDL3_VERSION=3.4.14`、`CLI11_VERSION=v2.7.2` に更新され、`CHANGES.md` の `develop` セクションに `UPDATE` エントリ (SDL3 / CLI11 / Boost / sora-cpp-sdk 2026.2.1 同期) が追記されている。Boost は `BOOST_VERSION=1.92.0` と正しいハッシュで更新されている
- `copy-from-sora-cpp-sdk.sh` / `copy-to-sora-cpp-sdk.sh` が `third_party` を含むよう修正され、`README.md` の手順が引数 2 個の実態に合致している
- 全プラットフォーム (`windows_x86_64` / `macos_arm64` / `ubuntu-24.04_x86_64` / `ubuntu-22.04_x86_64` / `raspberry-pi-os_armv8` / `ubuntu-22.04_armv8_jetson`) で `python3 run.py build <platform> --package` が通る。`run.py` の `AVAILABLE_TARGETS` は 7 個だが `macos_x86_64` は CI 対象外のため 6 個で判定する
- `e2e-test.yml` の E2E テスト (GitHub-hosted 3 matrix と self-hosted 4 種を含む) が `from_build: true` 経路で通る
- 下記の vendored コードに対する重要指摘 (13 件) が解消されているか確認され、結果が本 issue の解決方法に表形式で記録されている。各 finding について「解消状況 (解消 / 未解消)」「根拠」「対応先 issue」の 3 列を持つ表で記録する。確認手順は、各対象ファイルで該当シンボルをコードレビューで確認し、ランタイム挙動に依存する finding (VPL の `Data.UV` 等) は実機確認も含めて判定する。sora-cpp-sdk 由来のファイル (`hwenc_vpl` / `hwenc_nvcodec` / `hwenc_v4l2` / `third_party/libcamerac`) の finding は `2026.2.1` への同期後に取り込まれたコードで解消されているかを確認する。`hwenc_jetson` は momo 独自コードであり sora-cpp-sdk `2026.2.1` には存在しないため同期対象外で、`hwenc_jetson` を参照する finding (下記 2〜5 件目) は現行の momo コードに対して確認する。解消されていないものは既存の対応 issue がカバーしているかを確認し、カバーされていないもののみ別 issue を起票する。ファイルパスは `src/sora-cpp-sdk/` を基準にした sora-cpp-sdk 内の相対パス (`src/hwenc_*/*.cpp` 等) を示す。下記 13 件はすべて本 issue で初回確認する。なお `0020` (Jetson デコーダ self-join deadlock) / `0021` (Jetson エンコーダ SendEOS) / `0022` (V4L2 デコーダ memcpy 越境) / `0023` (libcamera UAF) は、下記リストとは別の finding を扱う既存 issue であり、下記リストには含まれない。`0020` が委譲した `CHUNK_SIZE` 超過 `memcpy` は下記リスト 4 件目に含まれる
  - VPL の NV12 で `Data.UV` 未設定 (`src/hwenc_vpl/vpl_video_decoder.cpp` の `Data.UV`。エンコーダ側は `Data.U` / `Data.V` を使用)
  - Jetson / V4L2 の `configured_bitrate_bps_` 未初期化比較 (`src/hwenc_jetson/jetson_video_encoder.cpp` / `src/hwenc_v4l2/v4l2_h264_encoder.cpp`)
  - IVF ヘッダ除去の size アンダーフロー (`src/hwenc_jetson/jetson_video_encoder.cpp` / `src/hwenc_nvcodec/nvcodec_video_encoder.cpp` / `src/hwenc_vpl/vpl_video_encoder.cpp`)
  - Jetson デコーダの `CHUNK_SIZE` 超過 memcpy (`src/hwenc_jetson/jetson_video_decoder.cpp`)
  - Jetson エンコーダの AV1 OBU サイズ 1 バイト固定読み (`src/hwenc_jetson/jetson_video_encoder.cpp`)
  - NvCodec エンコーダの `frameRateNum=0` 除算ゼロ (`src/hwenc_nvcodec/nvcodec_video_encoder.cpp`)
  - CUDA の `cuCtxPushCurrent` が例外パスで Pop されない (`src/hwenc_nvcodec/nvcodec_decoder_cuda.cpp` の `cuCtxPushCurrent` / `cuCtxPopCurrent`)
  - V4L2Runner が `POLLERR` / `HUP` を無視し破棄ハング (`src/hwenc_v4l2/v4l2_runner.cpp`)
  - V4L2 系のデバイスパス直書き (`src/hwenc_v4l2/v4l2_converter.cpp`)
  - libcamerac の Span dangling (`third_party/libcamerac/libcamerac.cpp`)
  - libcamera の mmap 戻り値が `MAP_FAILED` か未チェック (`src/hwenc_v4l2/libcamera_capturer.cpp` の `mmap` 戻り値)
  - NvCodec エンコーダのフレームサイズ未検証 GPU コピー (`src/hwenc_nvcodec/nvcodec_video_encoder.cpp`)
  - `src/hwenc_v4l2/v4l2_h264_decoder.cpp` の `Release()` 空実装

## 解決方法

未着手 (PR 作成後に追記する)
