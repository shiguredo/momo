# CUDA を 13.3.1-1 に上げる

- Created: 2026-08-28
- Completed: {YYYY-MM-DD}
- Branch: feature/update-cuda-13-3-1
- Polished: {YYYY-MM-DD}

## 目的

CUDA を `12.9.1-1` から `13.3.1-1` に上げる。sora-cpp-sdk はすでに 13.3.1-1 へ移行済みで、CUDA 13 での変更（`cuCtxCreate` の 4 引数化、Windows インストーラーでのコンポーネント分離、Maxwell / Pascal / Volta のサポート廃止）に momo も追随する。

## 現状

- `DEPS` の `CUDA_VERSION` は `12.9.1-1`
- `buildbase.py` の `install_cuda_windows` は `13.0.1-1` までの URL しか持たない。またコピーするのは `cuda_nvcc` と `cuda_cudart` だけで、CUDA 13 で分離された `cuda_crt`（`crt/host_config.h`）と `libnvvm`（`cicc`）を取り込んでいないため、`DEPS` の値を書き換えるだけでは Windows ビルドが通らない
- `CMakeLists.txt` の Linux CUDA `COMPILE_OPTIONS` は `--cuda-gpu-arch=sm_60`。CUDA 13 は sm_50 〜 sm_70 をサポートしないため、この arch 指定は使えなくなる
- Ubuntu x86_64 のビルドは clang-20 を使っている（`run.py` の `CMAKE_C_COMPILER` / `CMAKE_CXX_COMPILER` と `build.yml` の `llvm.sh` にハードコード）。sora-cpp-sdk では CUDA 13.3 を clang でコンパイルするために clang 22 が必要だった。Jetson と Raspberry Pi のクロスビルドは webrtc 管理下の clang を使うため影響しない
- `build.yml` は `setup-cuda-toolkit` に `cuda_version: 12.9.1` を直指定しており、Windows の CUDA キャッシュキーは `.v1` のまま
- vendored の `src/sora-cpp-sdk/src/cuda_context_cuda.cpp` と `src/sora-cpp-sdk/src/hwenc_nvcodec/nvcodec_video_encoder_cuda.cpp` は、`cuCtxCreate` を `#if CUDA_VERSION >= 13000` で CUDA 12 向け 3 引数版と分岐させている（コメントで「momo は CUDA 12 を使う」と述べている）

## 設計方針

- `DEPS` の `CUDA_VERSION` を `13.3.1-1` にする
- `buildbase.py` の `install_cuda_windows` に `13.3.1-1` の Windows インストーラー URL を追加し、`cuda_crt` と `libnvvm` をインストール対象に追加する（`13.` 系でこれらのディレクトリが見つからない場合はエラーにする）
- `CMakeLists.txt` の Linux CUDA `COMPILE_OPTIONS` を `--cuda-gpu-arch=sm_75`（Turing）に変更する
- Ubuntu x86_64 の clang を 20 から 22 に上げる（`run.py` の 2 か所と `build.yml` の `llvm.sh`）
- `build.yml` の `setup-cuda-toolkit` に渡す `cuda_version` を `13.3.1` に、Windows の CUDA キャッシュキーを `.v2` に上げる
- vendored ソースから CUDA 12 向け 3 引数分岐とそのコメントを削除し、sora-cpp-sdk と同じ呼び出しに戻す
- CUDA 13 で Maxwell / Pascal / Volta が使えなくなることは下位互換のない変更なので、`CHANGES.md` には `[UPDATE]`（CUDA のバージョン）と `[CHANGE]`（Pascal 世代以前のサポート廃止）を分けて記載する。`doc/FAQ.md` の動作確認が取れたビデオカード一覧にも Pascal 世代以前が利用できない旨を注記する
- CI と `DEPS` のバージョン二重管理の解消は行わず、値の更新のみ行う（一元化は他 issue の範囲）

## 完了条件

- `DEPS` の `CUDA_VERSION` が `13.3.1-1` になっている
- Windows / Ubuntu 22.04 / Ubuntu 24.04 のビルドが CI で通る
- 自ホストランナーの NVIDIA Video Codec E2E テストが通る（対象 GPU は Turing 以降、CUDA 13 を受け付けるドライバーであることが前提）
- `CHANGES.md` と `doc/FAQ.md` に Pascal 世代以前のサポート廃止が反映されている

## 参考

- sora-cpp-sdk で CUDA 13 移行時に踏んだ変更
  - `8bf52eac` CUDA のバージョンを 13.3.1-1 に上げる
  - `c5436508` CUDA 13.3 向けに clang 22 と cuda_crt を対応する
  - `9a20d6b7` CUDA 13 の cuCtxCreate_v4 に追従し GPU アーキテクチャを sm_75 に上げる
  - `89f14ca1` Windows で CUDA 13 の libnvvm に分離された cicc をインストール対象に含める
- CUDA 13.3.1 の配信状況（本 issue 作成時に確認済み）
  - NVIDIA リポジトリの `ubuntu2204` / `ubuntu2404` に `cuda-toolkit-13_13.3.1-1_amd64.deb` が存在する
  - `https://developer.download.nvidia.com/compute/cuda/13.3.1/local_installers/cuda_13.3.1_windows.exe` が応答する
