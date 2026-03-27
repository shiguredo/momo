# Ayame モードで --video-codec-type が無視される

Created: 2026-03-27
Model: Opus 4.6

## 概要

Ayame モードで `--video-codec-type` および `--audio-codec-type` オプションを指定しても、コーデック選択に反映されない。CLI でのパースとバリデーションは行われるが、実際の WebRTC 接続では Transceiver の `SetCodecPreferences` が呼ばれないため、デフォルトのコーデック (VP8) にフォールバックする。

## 再現手順

1. 2 つの momo インスタンスを Ayame モードで VP9 を指定して起動する

```bash
momo --fake-capture-device --resolution QVGA \
  --vp9-encoder software --vp9-decoder software \
  ayame --signaling-url wss://ayame-labo.shiguredo.app/signaling \
  --room-id test-room --video-codec-type VP9

momo --fake-capture-device --resolution QVGA \
  --vp9-encoder software --vp9-decoder software \
  ayame --signaling-url wss://ayame-labo.shiguredo.app/signaling \
  --room-id test-room --video-codec-type VP9
```

2. メトリクスの outbound-rtp から codecId を取得し、codec の mimeType を確認する
3. VP9 を指定したにもかかわらず `video/VP8` が使用される

## 期待する動作

C++ 版と同様に、`--video-codec-type VP9` を指定した場合は VP9 で接続される。

## 原因

1. `AyameConfig` 構造体に `video_codec_type` / `audio_codec_type` フィールドが存在しない
2. `add_transceivers` 関数で Transceiver の `SetCodecPreferences` を呼んでいない
3. C++ 版では `CreateRTCConnection` 内で `InitTracks` 後に `SetCodecPreferences` を呼び、指定コーデックを primary に、補助コーデック (rtx, red, ulpfec 等) を secondary として `SetCodecPreferences` に渡している

## 根拠

C++ 版の実装 (`src/ayame/ayame_client.cpp`):

- `SetCodecPreferences` 関数で `PeerConnectionFactory::GetRtpSenderCapabilities` / `GetRtpReceiverCapabilities` から共通コーデックを取得
- 指定コーデックを primary、補助コーデック (rtx, red, ulpfec, flexfec) を secondary としてフィルタリング
- `RtpTransceiverInterface::SetCodecPreferences` で SDP に反映

## 対応方針

1. `AyameConfig` に `video_codec_type: Option<String>` と `audio_codec_type: Option<String>` を追加する
2. `main.rs` でパースした値を `AyameConfig` に渡す
3. `add_transceivers` の後に `SetCodecPreferences` 相当の処理を追加する

### shiguredo_webrtc の API 状況

- FFI レベル: `webrtc_RtpTransceiverInterface_SetCodecPreferences` は存在する
- Rust ラッパー: `RtpTransceiver` に `set_codec_preferences` メソッドが**公開されていない**
- `PeerConnectionFactory` にも `get_rtp_sender_capabilities` / `get_rtp_receiver_capabilities` の Rust ラッパーが**公開されていない**

shiguredo_webrtc 側で Rust ラッパーの追加が必要な可能性がある。

## CI への影響

E2E テスト `test_ayame_mode_with_codec[VP9]` と `test_ayame_mode_with_codec[AV1]` が失敗する。
