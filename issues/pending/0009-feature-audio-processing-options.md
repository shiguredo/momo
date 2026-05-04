# 音声処理オプションの実装

## 概要

音声処理オプション (AEC/AGC/NS/ハイパスフィルター) と音声出力デバイスがパースされるが未使用。

## 現状

- `src/main.rs` で以下が `_` 付き変数に束縛されるだけ:
  - `_disable_echo_cancellation`
  - `_disable_auto_gain_control`
  - `_disable_noise_suppression`
  - `_disable_highpass_filter`
  - `_audio_output_device`
- MomoConfig に含まれず、WebRTC AudioProcessing に渡されていない

## 必要な実装

- `--disable-echo-cancellation`: WebRTC BuiltinAudioProcessing のエコーキャンセレーション無効化
- `--disable-auto-gain-control`: 自動ゲイン制御無効化
- `--disable-noise-suppression`: ノイズ抑制無効化
- `--disable-highpass-filter`: ハイパスフィルター無効化
- `--audio-output-device`: 音声出力デバイス指定
- MomoConfig に追加し各モードに渡す

## pending

shiguredo_webrtc (0.146.0-canary.4) の `AudioProcessingBuilder` に echo cancellation / AGC / noise suppression / highpass filter の設定 API が公開されていない。
`webrtc::BuiltinAudioProcessingBuilder` の C API ラッパーに `SetConfig` 相当の関数が追加されるまで実装不可。
