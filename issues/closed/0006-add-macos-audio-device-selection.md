# macOS でオーディオデバイス選択機能を追加する

- Created: 2025-10-14
- Completed: 2025-11-05
- Branch: feature/macos-audio-device-selection
- Polished: {YYYY-MM-DD}

## 目的

macOS で複数のマイク・スピーカーがあるとき、既定デバイス以外を指定して送受信できるようにする。あわせて利用可能なデバイスを確認する手段を提供する。

## 現状

macOS ではビデオデバイスは `--video-device`（当時）で指定できたが、オーディオ入出力はプラットフォーム既定に固定されていた。デバイス一覧を表示する `--list-devices` も macOS には無かった。

## 設計方針

- `--audio-input-device` / `--audio-output-device` でインデックスまたはデバイス名（完全一致、大文字小文字を区別しない）を指定する
- 指定が無ければシステムの既定デバイスを使う
- `--list-devices` でオーディオ入出力とビデオデバイスを列挙して終了する
- 列挙と選択は `webrtc::AudioDeviceModule` と `MacCapturer::GetVideoDeviceInfos` を使う

## 完了条件

- macOS で `--audio-input-device` / `--audio-output-device` が動作する
- `--list-devices` でオーディオとビデオの一覧が表示される
- 不正な指定では起動に失敗する

## 解決方法

PR #431 で以下を追加した。

- `src/util.cpp` に `--audio-input-device` / `--audio-output-device` / `--list-devices` を登録する
- `src/device_info.h` / `src/device_info.cpp` に `GetAudioDeviceInfos` を追加し、ADM から名前・GUID・インデックスを取得する
- `src/rtc/rtc_manager.cpp` の無名名前空間に `MatchDeviceIdentifier` / `ResolveDeviceIndex` / `SetAudioDevice` を置き、`RTCManager` 構築時に `SetRecordingDevice` / `SetPlayoutDevice` を呼ぶ
- `src/main.cpp` の `ListDevices` でオーディオ入出力と `MacCapturer::GetVideoDeviceInfos` によるビデオ一覧を表示する
- 指定値は `src/main.cpp` から `RTCManagerConfig::audio_input_device` / `audio_output_device` へ渡す
