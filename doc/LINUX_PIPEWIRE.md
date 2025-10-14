# Linux PipeWire オーディオデバイス実装

## 概要

Ubuntu 24.04 以降で採用されている PipeWire をネイティブサポートするためのオーディオデバイス実装。

## 重要: audio グループへの追加が必要

### なぜ audio グループが必要なのか

**PipeWire でも audio グループへの参加が必須です。**

PipeWire は「ユーザー権限で動作する」と言われていますが、これは **PipeWire サーバー自体が root 権限不要** という意味であり、**ハードウェアへのアクセスには audio グループが必要**です。

理由:
1. **PipeWire のアーキテクチャ**:
   - PipeWire サーバーはユーザーセッションで動作 (`/run/user/1000/pipewire-0`)
   - しかし実際のハードウェアアクセスは ALSA デバイス経由

2. **Linux のデバイスパーミッション**:
   - オーディオハードウェアは `/dev/snd/*` デバイスファイルとして公開される
   - これらのファイルのパーミッションは `crw-rw---- root audio` (例: `/dev/snd/controlC3`)
   - audio グループに属していないとアクセス不可

3. **実際の動作**:
   ```bash
   $ ls -l /dev/snd/controlC3
   crw-rw---- 1 root audio 116, 21 Oct 14 14:33 /dev/snd/controlC3

   $ cat /proc/asound/cards
   3 [YVC200]: USB-Audio - Yamaha YVC-200  # カーネルは認識している

   # audio グループなしの場合
   $ wpctl status
   Audio
    ├─ Devices:
    │                                           # デバイスが空

   # audio グループありの場合
   $ wpctl status
   Audio
    ├─ Devices:
    │      47. Yamaha YVC-200 [alsa]           # デバイスが見える
   ```

**結論**: PulseAudio と同様、PipeWire も底辺では Linux カーネルのデバイスパーミッションモデルに従うため、audio グループが必要です。

### audio グループへの追加方法

```bash
# ユーザーを audio グループに追加
sudo gpasswd -a $USER audio

# ログアウト/ログインまたはシステム再起動
# (グループ変更は新しいログインセッションでのみ反映される)

# グループメンバーシップを確認
groups
# 出力に "audio" が含まれていることを確認
```

**注意**: グループ変更後は必ずログアウト→再ログイン、または再起動してください。systemd ユーザーセッション全体を再起動する必要があります。

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
  - [x] AudioDeviceBuffer の作成と attach
  - [x] 10ms ネイティブ配信 (`PW_KEY_NODE_FORCE_QUANTUM`)
  - [x] リングバッファによる 10ms 境界の維持
  - [x] Sora への録音データ送信 (動作確認済み)
- [x] 再生 (playout) の実装
  - [x] PipeWire 再生ストリームの作成
  - [x] AudioDeviceBuffer からのオーディオデータ取得
  - [x] PipeWire へのデータ転送
  - [x] 10ms ネイティブ配信 (`PW_KEY_NODE_FORCE_QUANTUM`)
  - [x] リングバッファによる 10ms 境界の維持
  - [x] Sora からのオーディオ再生 (動作確認済み)
- [x] audio グループ権限の問題を解決

### 未実装の機能

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
            ├── pw_stream (録音ストリーム)
            └── pw_stream (再生ストリーム)
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
- 実際のストリーム作成時に選択したデバイスを使用

### 10ms ネイティブ配信の実装

WebRTC は 10ms (480 フレーム @ 48kHz) 単位でオーディオデータを処理します。PipeWire でこれを実現するため:

1. **`PW_KEY_NODE_FORCE_QUANTUM` プロパティ**: ストリーム作成時に quantum を 480 フレームに強制
   ```cpp
   pw_stream_new(pw_core_, "momo-recording",
                 pw_properties_new(
                     PW_KEY_MEDIA_TYPE, "Audio",
                     PW_KEY_MEDIA_CATEGORY, "Capture",
                     PW_KEY_MEDIA_ROLE, "Communication",
                     PW_KEY_NODE_FORCE_QUANTUM, "480",
                     nullptr));
   ```

2. **`SPA_PARAM_Latency` パラメータ**: pw_stream_connect で latency を明示
   ```cpp
   struct spa_latency_info latency_info = SPA_LATENCY_INFO(
       SPA_DIRECTION_INPUT,
       .min_quantum = 480,
       .max_quantum = 480);
   params[1] = spa_latency_build(&b, SPA_PARAM_Latency, &latency_info);
   ```

3. **リングバッファによる 10ms 境界の維持**: `OnRecStreamProcess` で受け取ったフレームをリングバッファに蓄積
   - 480 フレーム溜まったら WebRTC に配信
   - 480 フレーム未満の余剰フレームは次回へ繰り越し (データ欠落を防止)
   - chunk offset を考慮してサンプルポインタを正しく計算
   - 録音停止時にバッファをリセット (次回開始時に前回の端数が混入しないようにする)

この実装により:
- `PW_KEY_NODE_FORCE_QUANTUM` で 480 フレームのネイティブ配信を要求
- 万が一 480 以外のフレームが来てもリングバッファで 10ms 境界を維持
- データ欠落なし
- PulseAudio より低レイテンシを実現

## 今後の実装課題

### 優先度: 高

1. **エラーハンドリング**
   - PipeWire 接続エラーの検出と再接続
   - ストリーム作成失敗時のフォールバック
   - デバイス切断時の通知

2. **遅延の計測**
   - `PlayoutDelay()` の実装
   - PipeWire の遅延情報を取得

### 優先度: 低

3. **音量・ミュート制御**
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

# 4. デバイスを指定して Sora モードで実行 (録音機能実装済み、動作確認済み)
timeout 30 ./momo --no-video-device --audio-input-device 1 sora \
  --signaling-urls wss://example.com/signaling \
  --role sendonly --channel-id test --video false --audio true

# 動作確認結果:
# - AudioDeviceBuffer が正しく作成・attach される
# - 録音ストリームが STREAMING 状態 (状態 3) に遷移
# - OnRecStreamProcess コールバックが呼ばれ、audio_buffer_ が正しく設定されている
# - ICE 接続が確立され、オーディオデータが Sora に送信される
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
   - 問題: WebRTC は `AudioDeviceModuleImpl` 内部で AudioDeviceBuffer を作成するが、カスタム wrapper では自分で作成・attach する必要がある
   - 解決方法: `AudioDeviceModulePipeWire::Init()` で TaskQueueFactory と AudioDeviceBuffer を作成し、`AudioDeviceLinuxPipeWire::AttachAudioBuffer()` で設定
   - `audio_transport_` も保存して後で AudioDeviceBuffer に登録する方式に変更済み

## 実装見積もり

### 完了済み
- デバイス列挙と選択: 約 200 行
- 録音ストリーム実装: 約 250 行
- 再生ストリーム実装: 約 300 行
- 合計: 約 750 行

### 残作業
- エラーハンドリング: 100-200 行
- 遅延計測: 50-100 行
- 音量・ミュート制御: 100-200 行
