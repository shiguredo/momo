## --hw-mjpeg-decoder が無視される

Created: 2026-05-04
Model: Opus 4.7

## 概要

`--hw-mjpeg-decoder` オプションが CLI でパースされるが、内部で `_hw_mjpeg_decoder` (アンダースコア付き) に束縛されるだけで破棄される。カメラ MJPEG 入力のハードウェアデコード設定が反映されない。

## 再現手順

1. MJPEG をサポートするカメラを接続する
2. `momo --hw-mjpeg-decoder true p2p ...` で HW MJPEG デコーダを有効化して起動する
3. CPU 使用率や動作上、ソフトウェアデコードと差がない

## 期待する動作

momo (C++) と同様に、`--hw-mjpeg-decoder true` で MJPEG のハードウェアデコードが有効化される (Raspberry Pi の V4L2 M2M デコーダ等)。

## 根拠

- `src/main.rs:213` で `_hw_mjpeg_decoder: Option<String>` に束縛されているが利用されていない
- momo-cpp の `--hw-mjpeg-decoder` は V4L2 M2M デコーダ経由でカメラの MJPEG 出力をハードウェアデコードする

## 対応方針

- `_hw_mjpeg_decoder` のアンダースコアを除去し、MomoConfig に `hw_mjpeg_decoder: bool` を追加する
- カメラキャプチャパイプラインで V4L2 M2M デコーダ (libv4l2) または相当する仕組みで MJPEG → I420 変換を行う

## #0010 からの分離

元々 issue #0010 (pending、コーデック選択) に含まれていたが、#0010 の pending 理由は HW エンコーダ (Jetson #0011 / NVIDIA・Intel・VideoToolbox #0012) 未実装であり、MJPEG HW デコードはこれらとは独立して実装可能なため分離した。

## 参考

- momo-cpp の `--hw-mjpeg-decoder` 実装
- `src/main.rs:213` の現状の束縛
