# Linux PipeWire オーディオデバイス実装

## 概要

Ubuntu 24.04 以降で採用されている PipeWire をネイティブサポートするためのオーディオデバイス実装。

## 重要: audio グループへの追加が必要

PipeWire で USB オーディオデバイスにアクセスするには、ユーザーを `audio` グループに追加する必要があります。

```bash
# ユーザーを audio グループに追加
sudo usermod -a -G audio $USER

# ログアウト/ログインまたはシステム再起動

# グループメンバーシップを確認
groups
# 出力に "audio" が含まれていることを確認
```

`audio` グループに所属していない場合、`/dev/snd/controlC*` デバイスファイルにアクセスできず、PipeWire の ALSA モニターがデバイスを認識できません。

## 実装状況

### 完了した機能

- [x] PipeWire 初期化処理
- [x] デバイス列挙機能 (`--list-devices`)
- [x] オーディオ入力デバイス一覧の取得
- [x] オーディオ出力デバイス一覧の取得
- [x] コマンドラインオプション追加 (`--audio-input-device`, `--audio-output-device`)
- [x] デバイス選択機能 (インデックスまたは名前による指定)
- [x] CMake ビルド設定 (`USE_LINUX_PIPEWIRE_AUDIO`)
- [x] 録音 (recording) の実装
  - [x] PipeWire 録音ストリームの作成
  - [x] オーディオデータの取得と AudioDeviceBuffer への転送
  - [x] サンプルレート (48kHz)、チャンネル数 (2ch ステレオ) の設定
- [x] audio グループ権限の問題を解決

### 未実装の機能

- [ ] 再生 (playout) の実装
  - [ ] PipeWire 再生ストリームの作成
  - [ ] AudioDeviceBuffer からのオーディオデータ取得
  - [ ] PipeWire へのデータ転送
- [ ] エラーハンドリング
  - [ ] PipeWire 接続エラー時の処理
  - [ ] ストリーム作成失敗時の処理
  - [ ] デバイス切断時の処理
- [ ] 遅延 (latency) の計測と報告
- [ ] 音量制御機能 (現在は未実装として -1 を返している)
- [ ] ミュート制御機能 (現在は未実装として -1 を返している)

## アーキテクチャ

### ファイル構成

```
src/rtc/
├── audio_device_pipewire_linux.h       # PipeWire デバイス実装のヘッダー
├── audio_device_pipewire_linux.cpp     # PipeWire デバイス実装
├── audio_device_module_pipewire.h      # AudioDeviceModule wrapper ヘッダー
└── audio_device_module_pipewire.cpp    # AudioDeviceModule wrapper 実装
```

### クラス構成

```
AudioDeviceModule (interface)
    └── AudioDeviceModulePipeWire (wrapper)
            └── AudioDeviceLinuxPipeWire (implementation)
                    └── AudioDeviceGeneric (base class)
```

- `AudioDeviceModulePipeWire`: WebRTC の AudioDeviceModule インターフェースを実装する wrapper クラス
- `AudioDeviceLinuxPipeWire`: PipeWire を使った実際のデバイス制御を行うクラス

### PipeWire オブジェクト

```
pw_thread_loop (メインループ)
    └── pw_context (コンテキスト)
            ├── pw_core (コア接続)
            │       └── pw_registry (デバイスレジストリ)
            ├── pw_stream (録音ストリーム) ← 未実装
            └── pw_stream (再生ストリーム) ← 未実装
```

## 実装の詳細

### デバイス列挙

PipeWire registry を使ってデバイス情報を取得:

1. `pw_registry_add_listener()` でレジストリイベントをリスン
2. `OnRegistryGlobal()` コールバックで新しいデバイスを検出
3. `PW_TYPE_INTERFACE_Node` かつ `media.class` が `Audio/Source` または `Audio/Sink` のデバイスを収集
4. デバイス情報 (ID, 名前, 説明) を保存

### デバイス選択

- インデックスまたは名前でデバイスを指定可能
- `SetPlayoutDevice()` / `SetRecordingDevice()` で選択したデバイスのインデックスを保存
- 実際のストリーム作成時に選択したデバイスを使用 (未実装)

## 今後の実装課題

### 優先度: 高

1. **録音ストリームの実装**
   - `InitRecording()` で PipeWire ストリームを作成
   - `pw_stream_new()` でストリームを作成
   - `pw_stream_add_listener()` でイベントリスナーを追加
   - `pw_stream_connect()` で録音デバイスに接続
   - データコールバックで AudioDeviceBuffer にデータを転送

2. **再生ストリームの実装**
   - `InitPlayout()` で PipeWire ストリームを作成
   - AudioDeviceBuffer からデータを取得
   - PipeWire ストリームにデータを書き込み

3. **オーディオフォーマットの設定**
   - サンプルレート (48000 Hz が一般的)
   - チャンネル数 (1: モノラル, 2: ステレオ)
   - バッファサイズの設定

### 優先度: 中

4. **エラーハンドリング**
   - PipeWire 接続エラーの検出と再接続
   - ストリーム作成失敗時のフォールバック
   - デバイス切断時の通知

5. **遅延の計測**
   - `PlayoutDelay()` の実装
   - PipeWire の遅延情報を取得

### 優先度: 低

6. **音量・ミュート制御**
   - PipeWire の音量制御 API を使用
   - スピーカー/マイクの音量とミュート状態の取得・設定

## 参考実装

- WebRTC の ALSA 実装: `modules/audio_device/linux/audio_device_alsa_linux.cc`
- WebRTC の PulseAudio 実装: `modules/audio_device/linux/audio_device_pulse_linux.cc`
- macOS の実装: `src/rtc/rtc_manager.cpp` (デバイス選択部分)

## ビルド方法

```bash
# PipeWire を有効にしてビルド (デフォルトで ON)
python3 run.py build ubuntu-24.04_x86_64

# PipeWire を無効にしてビルド (ALSA を使用)
cmake -DUSE_LINUX_PIPEWIRE_AUDIO=OFF ...
```

## 動作確認

```bash
# 1. audio グループに所属していることを確認
groups
# 出力に "audio" が含まれていること

# 2. PipeWire でデバイスが認識されているか確認
wpctl status
# Audio Devices に USB デバイスが表示されること

# 3. momo でデバイス一覧を表示 (動作確認済み)
timeout 3 ./momo --no-video-device --list-devices

# 例: 出力
# === Available audio input devices ===
#   [0] HyperX QuadCast S Analog Stereo (alsa_input.usb-HP__Inc_HyperX_QuadCast_S-00.analog-stereo)
#   [1] Yamaha YVC-200 Mono (alsa_input.usb-Yamaha_Corporation_Yamaha_YVC-200-00.mono-fallback)
#   [2] Insta360 Link Mono (alsa_input.usb-Insta360_Insta360_Link-02.mono-fallback)
#
# === Available audio output devices ===
#   [0] HyperX QuadCast S Analog Stereo (alsa_output.usb-HP__Inc_HyperX_QuadCast_S-00.analog-stereo)
#   [1] Yamaha YVC-200 Mono (alsa_output.usb-Yamaha_Corporation_Yamaha_YVC-200-00.mono-fallback)

# 4. デバイスを指定して Sora モードで実行 (録音機能実装済み)
timeout 30 ./momo --no-video-device --audio-input-device 1 sora \
  --signaling-urls wss://example.com/signaling \
  --role sendonly --channel-id test --video false --audio true
```

**注意**:
- `--list-devices` は PipeWire 初期化に時間がかかるため、`timeout` を使用して 3 秒程度で終了させることを推奨
- Sora モードでのテストも `timeout` を使用して適切なタイミングで終了させること

## 既知の問題

### 解決済み: audio グループ権限の問題

**問題**: ユーザーが `audio` グループに所属していないと、USB オーディオデバイスが PipeWire で認識されない

**原因**: `/dev/snd/controlC*` デバイスファイルは `root:audio` 権限 (0660) で作成され、audio グループに所属していないユーザーはアクセスできない

**解決方法**: ユーザーを audio グループに追加し、ログアウト/ログインまたはシステム再起動を行う

```bash
sudo usermod -a -G audio $USER
# ログアウト/ログインまたはシステム再起動
```

### その他の問題

1. PipeWire 設定ファイルの警告
   - `can't load config client.conf: No such file or directory`
   - 動作には影響なし (警告レベル)

2. AudioDeviceBuffer not attached エラー (解決済み)
   - `RegisterAudioCallback()` が `AttachAudioBuffer()` より先に呼ばれる
   - `audio_transport_` を保存して後で登録する方式に変更済み

## 実装見積もり

### 完了済み
- 録音ストリーム実装: 約 250 行 (完了)
- デバイス列挙と選択: 約 200 行 (完了)

### 残作業
- 再生ストリーム実装: 300-500 行 (録音ストリームと同様の実装パターン)
- エラーハンドリング: 100-200 行
- 残り合計: 400-700 行程度

実装期間: 残り 1-2 週間程度
