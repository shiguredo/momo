# recvonly のときに不要な送信トラックを作らないようにする

- Created: 2026-09-11
- Completed: {YYYY-MM-DD}
- Branch: feature/change-recvonly-skip-send-track
- Polished: {YYYY-MM-DD}
- Reporter: @voluntas

## 目的

Ayame / Sora モードで recvonly を指定したとき、`--no-audio-device` / `--no-video-input-device` を指定しない限り送信トラックが生成され、キャプチャデバイスも開かれる。recvonly では送信しないため、これらは不要なリソース消費になる。recvonly のときは送信トラックを生成しないようにする。

## 現状

- `src/main.cpp` は `--no-video-input-device` の有無にかかわらずビデオキャプチャ (`capturer`) を生成して `RTCManager` に渡す。`--no-audio-device` でない限り ADM も生成する
- `src/rtc/rtc_manager.cpp` の `RTCManager` コンストラクタは `!config_.no_audio_device` のとき `audio_track_` を、`video_track_source && !config_.no_video_device` のとき `video_track_` を生成する。direction は参照しない
- `RTCManager::InitTracks` は direction が recvonly の場合 `kRecvOnly` の transceiver を追加するだけで、`audio_track_` / `video_track_` は追加しない。しかしトラック自体はコンストラクタで既に生成されている
- Sora モードは `src/sora/sora_client.cpp` が `role == "recvonly"` のとき direction を `"recvonly"` にして `InitTracks` を呼ぶ。Ayame モードは `src/ayame/ayame_client.cpp` が `config_.direction` をそのまま渡す
- `--no-audio-device` / `--no-video-input-device` を指定すればトラックは生成されないが、recvonly のためだけに指定するのは本来不要

## 設計方針

- recvonly のときは送信トラック (`audio_track_` / `video_track_`) を生成しない
- 送信トラックを生成しないのに合わせて、キャプチャデバイス (ビデオキャプチャ / ADM の録音) も開かないようにする
- direction は Sora / Ayame モードで起動時に確定するため、`RTCManager` に direction を渡すか、`src/main.cpp` でキャプチャを生成しないようにする。どちらで扱うかを決める
- P2P モードは direction を持たないため従来どおりとする
- 送信トラックが無い状態での `/mute` や受信制御の扱いを、既存の実装 (送信トラックが無ければ `400` を返す) と矛盾しないようにする

## 完了条件

- Ayame / Sora モードで recvonly を指定したとき、`--no-audio-device` / `--no-video-input-device` を指定しなくても `audio_track_` / `video_track_` が生成されない
- recvonly でビデオキャプチャ / ADM の録音が開かれない
- sendrecv / sendonly では従来どおり送信トラックが生成され、配信できる
- P2P モードの挙動が変わらない

## 解決方法

未着手 (方式が決まり PR 作成後に追記する)

## pending にした理由

機能的な不具合ではなく、recvonly で不要な送信トラックとキャプチャデバイスを生成しているリソースの無駄遣いである。direction を `RTCManager` にどう渡すか、キャプチャデバイスをどこで生成しないようにするかなど実装箇所の整理が必要で、不急のため pending とする。
