# Linux でオーディオデバイス選択機能を追加する

- Created: 2025-11-06
- Completed: 2025-11-06
- Branch: feature/linux-audio-device-selection
- Polished: {YYYY-MM-DD}

## 目的

Linux でも macOS と同様にオーディオ入出力デバイスを指定し、`--list-devices` でオーディオデバイスを確認できるようにする。pipewire-pulse 経由を前提に、選択経路を PulseAudio API に統一する。

## 現状

Linux のビデオデバイスは `--list-devices` と `--video-input-device` で扱えていたが、オーディオは ADM のプラットフォーム既定に任せており、デバイス指定も一覧表示も無かった。ALSA 専用の選択コードが残っていると PulseAudio / PipeWire 環境と経路が分裂する。

## 設計方針

- Linux の `AudioLayer` を常に `webrtc::AudioDeviceModule::kLinuxPulseAudio` にする
- デバイス指定と列挙の実装は macOS と同じ `GetAudioDeviceInfos` / `SetAudioDevice` を使う
- 指定方法はインデックスまたはデバイス名（完全一致、大文字小文字を区別しない）
- `--list-devices` の既存ビデオ一覧に、オーディオ入出力一覧を足す

## 完了条件

- Linux で `--audio-input-device` / `--audio-output-device` が動作する
- `--list-devices` にオーディオ入出力が表示される
- ALSA 専用のデバイス選択コードが残っていない
- ADM が `kLinuxPulseAudio` を使う

## 解決方法

PR #433 で以下を変更した。

- `src/rtc/rtc_manager.cpp` と `src/main.cpp` の `ListDevices` で Linux の `AudioLayer` を `kLinuxPulseAudio` に固定する
- `src/util.cpp` の `--audio-input-device` / `--audio-output-device` を Linux でも有効にする（`__APPLE__` または `__linux__`）
- macOS で入れた `SetAudioDevice` を Linux でも同じ条件コンパイルで利用する
- `src/main.cpp` の `ListDevices` で `GetAudioDeviceInfos` によるオーディオ一覧を先に出し、続けて `sora::EnumV4L2CaptureDevices` / `sora::FormatV4L2Devices` でビデオ一覧を出す
- `doc/LINUX_AUDIO_DEVICE.md` を追加する
