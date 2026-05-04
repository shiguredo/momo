## --audio-output-device が無視される

Created: 2026-05-04
Model: Opus 4.7

## 概要

`--audio-output-device` オプションが CLI でパースされるが、内部で `_audio_output_device` (アンダースコア付き) に束縛されるだけで破棄される。MomoConfig にも含まれず、音声出力デバイスの選択に反映されない。

## 再現手順

1. `momo --list-devices` で利用可能な音声出力デバイスを確認する
2. `momo --audio-output-device <name> p2p ...` で特定のデバイスを指定して起動する
3. 指定したデバイスではなく OS のデフォルト音声出力デバイスから再生される

## 期待する動作

momo (C++) と同様に、`--audio-output-device` で指定したデバイスから音声が再生される。

## 根拠

- `src/main.rs:171` で `_audio_output_device: Option<String>` に束縛されているが利用されていない
- momo-cpp の `--audio-output-device` は WebRTC の音声デバイス選択に渡される

## 対応方針

- `_audio_output_device` のアンダースコアを除去し、MomoConfig に `audio_output_device: Option<String>` を追加する
- 各モードの音声処理パイプラインに渡し、cpal 等で出力デバイスを選択する

## #0009 からの分離

元々 issue #0009 (pending、音声処理オプション) に含まれていたが、#0009 の pending 理由は shiguredo_webrtc の `BuiltinAudioProcessing` API 不足であり、音声出力デバイス選択はそれとは独立して実装可能なため分離した。

## 参考

- momo-cpp の `--audio-output-device` 実装
- `src/main.rs:171` の現状の束縛
