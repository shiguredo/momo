# 依存バージョンの二重管理 (DEPS vs CI) を一元化する

- Created: 2026-08-19
- Completed: {YYYY-MM-DD}
- Branch: feature/refactor-deps-single-source
- Polished: {YYYY-MM-DD}

## 目的

依存ライブラリのバージョンが `DEPS` と CI (`.github/workflows/*.yml`) で二重管理されており、更新漏れリスクがある。DEPS を更新しても CI 側が追従しないと、ビルドとテスト・実行環境のバージョンが乖離する。依存バージョンを一元管理する。

## 現状 (二重管理箇所)

- `DEPS` の `OPENH264_VERSION=v2.6.0` と `e2e-test.yml` (28 行) の `OPENH264_VERSION: 2.6.0` (`v` 有無も不統一)
- `DEPS` の `CUDA_VERSION=12.9.1-1` と `build.yml` (154 行) の `cuda_version: 12.9.1` (Windows 側は DEPS 参照、Ubuntu 側だけ別管理)
- `run.py` / `build.yml` の clang-20 がハードコード (apt の提供バージョンが変わると両ファイルを同期変更する必要がある)
- `buildbase.py` の CUDA URL 分岐がバージョン依存 (DEPS 変更と同時編集が必要)

## 設計方針

- `e2e-test.yml` の `OPENH264_VERSION` を DEPS 参照に変える (v 表記も統一)
- `build.yml` の `cuda_version` を DEPS から取得する (Windows 側と同様の仕組み)
- clang のバージョン管理を一元化する
- 更新時に二重管理が残らない構造にする

## 完了条件

- 依存バージョンが単一のソース (DEPS) で管理されている
- DEPS 更新時に CI 側の追従漏れが構造的に起きない
- 全プラットフォームのビルド・E2E テストが通る

## 解決方法

未着手 (PR 作成後に追記する)
