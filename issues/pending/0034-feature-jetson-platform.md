## Jetson プラットフォーム対応

Created: 2026-05-04
Model: Opus 4.7

## 概要

NVIDIA Jetson プラットフォーム (aarch64) のクロスコンパイル・パッケージング・実行対応。HW エンコーダ実装 (#0011) とは別の、プラットフォーム全体のサポートに関する課題。

## 現状

- momo-rs に Jetson 向けクロスコンパイルの仕組みがない
- リリースアーティファクトに Jetson バイナリが含まれていない

## momo の対応状況

- ビルドターゲット: `ubuntu-22.04_armv8_jetson` (`run.py AVAILABLE_TARGETS`)
- `python3 run.py build` + multistrap で Jetson 向け rootfs を構築
- L4T (Linux for Tegra) のライブラリパス・依存ライブラリを解決
- nvbuf_utils / TensorRT 等の Jetson 固有 SDK と連携

## 必要な検討

- クロスコンパイル方法 (cross-rs / cargo-zigbuild / 手動 sysroot 等)
- L4T sysroot の準備手段 (multistrap 相当が Rust 側でどう実現できるか)
- shiguredo_webrtc の Jetson ビルド対応
- Jetson HW エンコーダ実装 (#0011) との統合

## pending 理由

クロスコンパイル環境の構築方針 (どのツールを使うか、L4T sysroot の取得方法、CI での再現性) について設計判断が必要。HW エンコーダ実装 (#0011) と密結合する部分もあり、両者の進め方を併せて検討する必要がある。

## 関連

- #0011 H.264 HW エンコード (Jetson) の実装 — Jetson HW エンコーダ単体の課題
- #0033 Windows プラットフォーム対応 — 同種のプラットフォーム拡張

## 参考

- momo-cpp の Jetson ビルド設定 (`run.py` の ubuntu-22.04_armv8_jetson)
