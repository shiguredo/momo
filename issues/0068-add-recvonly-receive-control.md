# recvonly のときに音や映像の受信を制御できるようにする

- Created: 2026-09-11
- Completed: {YYYY-MM-DD}
- Branch: feature/add-recvonly-receive-control
- Polished: {YYYY-MM-DD}
- Reporter: @voluntas

## 目的

recvonly で接続したときに、音や映像を受け取らない、あるいは再生しない方法が無い。特に音声はスピーカーがある限り再生され、拒否する方法が無い。映像は `--use-sdl` を指定しなければ表示されないため、指定しないことが実質的な受信の無効化になっているが、音声には相当する手段が無い。受信を止める hard mute と、受信はするが再生しない soft mute を用意する。

## 現状

- `--role recvonly` (Sora モード) / `--direction recvonly` (Ayame モード) を指定すると、`RTCManager::InitTracks` が `kRecvOnly` の audio / video transceiver を追加する (`src/rtc/rtc_manager.cpp`)
- 受信した音声は ADM の playout で再生される。`--no-audio-device` を指定すると ADM が `webrtc::AudioDeviceModule::kDummyAudio` になり再生されないが、音声デバイス自体を無効にする指定であり、送信側にも影響し、接続中に切り替えることもできない
- 受信した映像は `--use-sdl` を指定しなければ表示されない。`--use-sdl` を指定しないことが実質的な soft mute になっている
- `/mute` / `/mute/status` (Sora モード) は `RTCConnection::SetAudioEnabled` / `SetVideoEnabled` を通じてローカルの送信トラックを対象にしており、受信側は制御できない
- recvonly ではローカル送信トラックが無いため `/mute` は `400 Bad Request` を返す

## 設計方針

- 受信に対して soft mute と hard mute の 2 方式を用意する
  - soft mute: 受信は継続するが再生しない (音声は playout を停止し、映像は SDL に渡さない)
  - hard mute: 受信しない (該当メディアの transceiver の direction を `kInactive` にする、または transceiver を追加しない)
- 音声と映像を個別に制御できるようにする
- CLI オプションから制御できるようにする。Sora モードの `/mute` API からも制御できるかは、既存の `/mute` (送信の mute) の意味と衝突しない API の形を決めたうえで判断する
- `--no-audio-device` / `--use-sdl` の既存の挙動は変えず、受信制御を別のオプションとして追加する
- soft mute と hard mute は接続中に切り替えられるようにする

## 完了条件

- recvonly で音声のみ、映像のみ、両方を hard mute できる
- recvonly で音声のみ、映像のみ、両方を soft mute できる
- soft mute / hard mute を解除して受信・再生を再開できる
- hard mute 中は該当メディアが受信されない (stats で確認する)
- 上記を E2E テストで確認できる

## 解決方法

{YYYY-MM-DD} に追記する
