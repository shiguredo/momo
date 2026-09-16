# CUDA を 12.9.1-1 に上げる

- Created: 2025-10-09
- Completed: 2025-10-09
- Branch: feature/improve-gha-cuda
- Polished: {YYYY-MM-DD}

## 目的

CUDA を 12.9.1-1 に上げ、Kepler / Maxwell 世代をドロップして Pascal 以降 (`sm_60`) を対象にする。あわせて Ubuntu 22.04 / 24.04 で CUDA ツールキットを入れられるようにする。

## 現状

`DEPS` の `CUDA_VERSION` は `11.8.0-1` だった。Linux の CUDA コンパイルは `--cuda-gpu-arch=sm_35`（Kepler）で、Kepler は CUDA 10 まで、Maxwell (`sm_50`) は CUDA 11 までのサポートである。CI の Ubuntu 向け CUDA 導入も 12.9 に合わせて更新する必要があった。

## 設計方針

- Windows / Linux とも `DEPS` の `CUDA_VERSION` を `12.9.1-1` にする
- GPU アーキテクチャを `sm_60` にする
- libc++ との組み合わせ向けに `_ALLOW_UNSUPPORTED_LIBCPP` を付ける
- CI の CUDA 導入は `shiguredo/github-actions` の `setup-cuda-toolkit` に寄せ、Ubuntu 22.04 / 24.04 のキーリングとパッケージを使う

## 完了条件

- `DEPS` の `CUDA_VERSION` が `12.9.1-1` になっている
- Linux の CUDA コンパイルオプションが `sm_60` と `_ALLOW_UNSUPPORTED_LIBCPP` を含む
- Ubuntu 22.04 / 24.04 の CUDA ビルドが CI で通る

## 解決方法

PR #428 で以下を変更した。

- `DEPS` の `CUDA_VERSION` を `12.9.1-1` にする
- `buildbase.py` に CUDA 12.9.1 の Windows インストーラ URL を追加する
- `CMakeLists.txt` の Linux CUDA `COMPILE_OPTIONS` を `--cuda-gpu-arch=sm_60` と `-D_ALLOW_UNSUPPORTED_LIBCPP` に更新する
- `.github/workflows/build.yml` の Ubuntu CUDA 導入を `shiguredo/github-actions` の `setup-cuda-toolkit` に切り替え、`cuda_version: 12.9.1` を渡す
