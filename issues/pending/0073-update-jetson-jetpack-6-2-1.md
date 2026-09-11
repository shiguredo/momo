# Jetson JetPack 6.2.1 に対応する

- Created: 2026-09-11
- Completed: {YYYY-MM-DD}
- Branch: feature/update-jetson-jetpack-6-2-1
- Polished: {YYYY-MM-DD}
- Reporter: @miosakuma

## 目的

Jetson 向けバイナリは現在 JetPack 6.0 (L4T r36.3) を前提としており、`doc/SETUP_JETSON.md` も JetPack 6.0.0 のみの利用を前提としている。JetPack 6.2.1 (L4T 36.4.4) が公開されているため対応する。6.0 から 6.1、6.2.1 と順に上げて差分を確認する。

## 現状

- `sysroot/ubuntu-22.04_armv8_jetson.json` の NVIDIA リポジトリは suite `r36.3` を指定しており、JetPack 6.0 相当
- `doc/SETUP_JETSON.md` は「NVIDIA Jetson シリーズでは JetPack 6.0.0 のみの利用を前提としています」と記載している
- CUDA は issue 0055 で 13.3.1 に更新済み。JetPack の更新に伴う CUDA / L4T / ツールチェーンの差分を確認する必要がある
- JetPack 6 では `--hw-mjpeg-decoder` が有効だと H.264 が送信できない問題があり (`doc/SETUP_JETSON.md` の既知の問題、0074 で扱う)、JetPack の更新で解消する可能性がある
- ビルドは `ubuntu-22.04_armv8_jetson` の sysroot を `sysroot_builder.py` で生成してクロスコンパイルし、CI の `build-ubuntu` matrix でビルドする

## 設計方針

- `sysroot/ubuntu-22.04_armv8_jetson.json` の suite とパッケージを JetPack 6.2.1 (L4T 36.4.4) に合わせて更新する。6.1 から順に上げて破損がないか確認する
- JetPack 6.2.1 で必要になる CUDA / L4T のバージョン差分を確認し、`buildbase.py` / `run.py` / CI のセットアップを更新する
- Jetson 実機 (AGX Orin / Orin NX) でビルドしたバイナリの起動と配信を確認する。E2E は自ホストランナーが使える場合は実行し、使えない場合は手動確認とする
- `doc/SETUP_JETSON.md` の対応バージョンと既知の問題の記載を更新する
- 対応できない場合は理由を明記して pending を維持する

## 完了条件

- `sysroot/ubuntu-22.04_armv8_jetson.json` が JetPack 6.2.1 に対応している
- `ubuntu-22.04_armv8_jetson` のビルドが通り、Jetson 実機で起動する
- `doc/SETUP_JETSON.md` の前提バージョンが JetPack 6.2.1 になっている
- JetPack 6 の `--hw-mjpeg-decoder` の既知の問題が解消したか確認され、結果が 0074 に反映されている

## 解決方法

未着手 (JetPack 6.2.1 対応後に追記する)

## pending にした理由

JetPack の更新は sysroot の再生成、CUDA / L4T の追従、Jetson 実機での検証が必要で、すぐには対応できない。当面は JetPack 6.0 を使い、余裕のあるタイミングで 6.1 から順に上げるため pending とする。
