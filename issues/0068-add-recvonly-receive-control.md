# recvonly のときに音や映像の受信を制御できるようにする

- Created: 2026-09-11
- Completed: {YYYY-MM-DD}
- Branch: feature/add-recvonly-receive-control
- Polished: 2026-09-14
- Reporter: @voluntas

## 目的

recvonly で接続したときに、音や映像を受け取らない、あるいは再生しない方法が無い。特に音声はスピーカーがある限り再生され、拒否する方法が無い。映像は `--use-sdl` を指定しなければ表示されないため、指定しないことが実質的な soft mute (受信はするが表示しない) になっているが、音声には相当する手段が無い。受信はするが再生しない soft mute と、接続時にメディアを受信しない hard mute を用意する。

## 現状

- `--role recvonly` (Sora モード) / `--direction recvonly` (Ayame モード) を指定すると、`RTCManager::InitTracks` が `kRecvOnly` の audio / video transceiver を追加する (`src/rtc/rtc_manager.cpp`)
- 受信した音声は ADM の playout で再生される。`--no-audio-device` を指定すると ADM が `webrtc::AudioDeviceModule::kDummyAudio` になり再生されないが、音声デバイス自体を無効にする指定であり、送受信の両方に影響し、接続中に切り替えることもできない
- 受信した映像は `--use-sdl` を指定しなければ表示されない。`--use-sdl` を指定しないことが実質的な soft mute になっている (受信はするが表示しない)
- `/mute` / `/mute/status` (Sora モード) は `RTCConnection::SetAudioEnabled` / `SetVideoEnabled` を通じてローカルの送信トラックを対象にしており、受信側は制御できない
- recvonly ではローカル送信トラックが無いため `/mute` は `400 Bad Request` を返す
- libwebrtc の public API で受信を止められるのは transceiver の解放系のみである。`RtpTransceiver::SetDirectionWithError` (`SetDirection`) は direction の変更を次回の offer / answer まで反映しないため、接続中に `kInactive` へ変更しても受信は止まらない。`StopStandard` (stop()) は即時に受信を止めるが不可逆であり、mute の解除と両立しない

## 設計方針

- 受信に対して soft mute と hard mute の 2 方式を用意する
  - soft mute: 受信は継続するが再生しない (音声は ADM の playout を停止し、映像は SDL に渡さない)。接続中に切り替えられるようにする
  - hard mute: 受信しない (該当メディアの transceiver を追加しない)。起動時に CLI オプションで指定し、接続中には切り替えない
- 接続中の hard mute は本 issue の対象外とする。受信の停止には再交渉が必要であり、Sora モードでは server 駆動の re-offer / update (`src/sora/sora_client.cpp`) に依存して momo が自発的に再交渉できない。Ayame モードも再交渉フローを持たないため同様。再交渉による hard mute は将来課題とする
- 音声と映像を個別に制御できるようにする
- CLI オプションから制御できるようにする。Sora モードの `/mute` API は送信側を対象とする既存 API であり意味が異なるため受信制御には使わず、`400 Bad Request` の挙動も変えない。受信制御の HTTP API 化は本 issue の対象外とする
- `--no-audio-device` / `--use-sdl` の既存の挙動は変えず、受信制御を別のオプションとして追加する
- 音声の soft mute では ADM を保持して `StartPlayout` / `StopPlayout` を切り替える (ADM は現在 `RTCManager` のローカル変数として保持されているためメンバ変数化が必要)
- `--no-audio-device` 指定時は音声が元々再生されないため、音声の soft mute は no-op でよい
- 送信トラックを作らない改修 (0070-change-recvonly-skip-send-track、pending) とは独立に進められる

## 完了条件

- recvonly で音声のみ、映像のみ、両方を hard mute して接続できる
- recvonly で音声のみ、映像のみ、両方を soft mute できる
- soft mute を解除して再生を再開できる
- soft mute 中も該当メディアの受信が継続する (inbound-rtp 統計の `packetsReceived` が増加することを確認できる)
- hard mute 中は該当メディアが受信されない (該当メディアの inbound-rtp 統計が存在しないことを確認できる)
- 上記を E2E テストで確認できる (再生されないこと自体は stats で観測できないため、soft mute は受信が継続することを確認する)
- 既存の `--no-audio-device` / `--use-sdl` の挙動が変わらない

## 解決方法

{YYYY-MM-DD} に追記する
