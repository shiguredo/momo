# 依存バージョンの二重管理 (DEPS vs CI) を一元化する

- Created: 2026-08-19
- Completed: {YYYY-MM-DD}
- Branch: feature/refactor-deps-single-source
- Polished: 2026-09-13

## 目的

依存ライブラリのバージョンが `DEPS` と CI (`.github/workflows/*.yml`) で二重管理されており、更新漏れリスクがある。DEPS を更新しても CI 側が追従しないと、ビルドとテスト・実行環境のバージョンが乖離する。依存バージョンを一元管理する。

## 現状 (二重管理箇所)

- `DEPS` の `OPENH264_VERSION=v2.6.0` と `e2e-test.yml` の `env.OPENH264_VERSION: 2.6.0` (`v` 有無も不統一。`v` 有りは `buildbase.py` の `install_openh264` が git tag として使う)
- `DEPS` の `CUDA_VERSION=12.9.1-1` と `build.yml` の `setup-cuda-toolkit` に渡す `cuda_version: 12.9.1` (Windows 側は `build.yml` の `build-windows` ジョブと `run.py` が DEPS 参照、Ubuntu 側だけ別管理)
- `run.py` の `-DCMAKE_C_COMPILER=clang-20` / `-DCMAKE_CXX_COMPILER=clang++-20` と `build.yml` の `llvm.sh 20` がハードコード (apt の提供バージョンが変わると `run.py` の VPL インストールと本体ビルドの 2 箇所 + `build.yml` を同時に変更する必要がある)
- `buildbase.py` の `install_cuda_windows` の CUDA URL 分岐がバージョン依存 (DEPS 変更と同時編集が必要)

## 設計方針

- `DEPS` を単一のソースにする。ワークフローは checkout 後に `DEPS` の `KEY=VALUE` をパースして `GITHUB_OUTPUT` に出力するステップを追加し、`${{ steps.versions.outputs.* }}` で参照する (Ubuntu 側は bash、Windows の `build-windows` ジョブの Get Versions と同様の仕組み)
- OpenH264: `DEPS` の `OPENH264_VERSION=v2.6.0` はそのまま (`install_openh264` の git tag 参照のため)。`e2e-test.yml` では `v` を除いた `2.6.0` を `download-openh264` の `openh264_version` に渡す (この action は内部で `v${VERSION}` に組み立てるため)
- CUDA: `DEPS` の `CUDA_VERSION` (`12.9.1-1`) を `build.yml` の `setup-cuda-toolkit` にそのまま渡す (action は `-1` の有無を自動判定するため)
- clang: `DEPS` に `CLANG_VERSION=20` を追加する。`run.py` の 2 箇所 (VPL インストールと本体ビルド) を `deps["CLANG_VERSION"]` 参照にし、`build.yml` の `llvm.sh` には DEPS パース結果を渡す
- `buildbase.py` の `install_cuda_windows` の CUDA インストーラー URL 分岐は本 issue の変更対象外とする。URL はバージョンごとに NVIDIA の配布形式が異なり自動生成できないためで、未対応バージョンは例外で明示的に失敗する (サイレントな乖離にはならない)。DEPS の更新と合わせた URL 追加は CUDA の更新系 issue (例: 0055) で行う
- 更新時に二重管理が残らない構造にする

## 完了条件

- 依存バージョン (OpenH264 / CUDA / clang) が単一のソース (DEPS) で管理されている
- DEPS 更新時に CI 側の追従漏れが構造的に起きない
- CI の全ビルド・E2E テストが通る

## 解決方法

未着手 (PR 作成後に追記する)
