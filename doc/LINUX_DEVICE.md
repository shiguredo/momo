# Linux でのデバイスアクセス

## 概要

Linux 環境で momo を使用してオーディオデバイスやビデオデバイスにアクセスするには、適切なシステム設定とユーザー権限が必要です。

## 必要なソフトウェア

### オーディオシステム

オーディオデバイスを使用するには、PulseAudio または ALSA が必要です。

#### PulseAudio のインストール（推奨）

```bash
sudo apt install pulseaudio
```

#### ALSA のインストール

```bash
sudo apt install alsa-base alsa-utils
```

最近の Ubuntu では PulseAudio（または後継の PipeWire）が標準的です。デスクトップ環境では通常プリインストールされていますが、サーバー環境やヘッドレス環境では手動インストールが必要な場合があります。

### PulseAudio の起動

```bash
pulseaudio --start
```

systemd 経由で起動する場合:

```bash
systemctl --user start pulseaudio
```

## ユーザー権限の設定

Linux では、オーディオデバイスとビデオデバイスへのアクセスにグループ権限が必要です。

### 必要なグループ

- `audio`: オーディオデバイス（`/dev/snd/*` など）へのアクセスに必要
- `video`: ビデオデバイス（`/dev/video*` など）へのアクセスに必要

### グループへの参加

```bash
sudo usermod -a -G audio,video $USER
```

### 権限の確認

デバイスファイルの権限を確認:

```bash
# オーディオデバイス
ls -l /dev/snd/

# ビデオデバイス
ls -l /dev/video*
```

典型的な出力例:

```
crw-rw---- 1 root video 81, 0 Oct 14 07:50 /dev/video0
```

この例では、`video` グループに所属するユーザーのみが読み書き可能です。

### 現在のグループの確認

```bash
groups
```

### グループ権限の反映

グループに参加した後は、以下のいずれかの方法で権限を反映させる必要があります:

1. ログアウトして再ログイン（推奨）
2. 新しいシェルセッションを開始:

   ```bash
   newgrp audio
   newgrp video
   ```

3. 一時的にグループ権限で実行:

   ```bash
   sg audio -c "sg video -c 'コマンド'"
   ```

## デバイス一覧の確認

### momo でのデバイス確認

```bash
./momo --list-devices
```

正常に動作している場合の出力例:

```
=== Available audio input devices ===

  [0] default: Insta360 Link Mono
  [1] HyperX QuadCast S Analog Stereo
  [2] Yamaha YVC-200 Mono

=== Available audio output devices ===

  [0] default: Yamaha YVC-200 Mono
  [1] HyperX QuadCast S Analog Stereo

=== Available video devices ===

  [0] Insta360 Link: Insta360 Link (/dev/video0)
```

### PulseAudio でのデバイス確認

音声入力デバイス:

```bash
pactl list sources short
```

音声出力デバイス:

```bash
pactl list sinks short
```

### ALSA でのデバイス確認

```bash
aplay -l    # 出力デバイス
arecord -l  # 入力デバイス
```

### カーネルが認識しているサウンドカード

```bash
cat /proc/asound/cards
```

## トラブルシューティング

### オーディオデバイスが表示されない

**症状:**

```
Warning: AudioDeviceModule::Init failed (error code: -1). Audio device enumeration is not available.
Note: This may occur if PulseAudio or ALSA is not running.
```

**原因と対処:**

1. PulseAudio が起動していない

   ```bash
   pulseaudio --start
   ```

2. ユーザーが `audio` グループに所属していない

   ```bash
   sudo usermod -a -G audio $USER
   # ログアウトして再ログイン、または newgrp audio
   ```

3. PulseAudio がハードウェアを認識していない

   ```bash
   pulseaudio --kill
   pulseaudio --start
   ```

### ビデオデバイスが表示されない（sudo では動作する）

**原因:**

ユーザーが `video` グループに所属していないため、`/dev/video*` へのアクセス権がありません。

**対処:**

```bash
sudo usermod -a -G video $USER
# ログアウトして再ログイン
```

### Dummy デバイスしか表示されない

**原因:**

- PulseAudio が ALSA のハードウェアデバイスを認識していない
- ユーザー権限の問題

**対処:**

1. グループ権限を確認:

   ```bash
   groups
   # audio と video が含まれているか確認
   ```

2. PulseAudio を適切な権限で再起動:

   ```bash
   sg audio -c "pulseaudio --kill; pulseaudio --start"
   ```

3. デバイスが認識されているか確認:

   ```bash
   cat /proc/asound/cards
   ```

## 参考情報

- PulseAudio: <https://www.freedesktop.org/wiki/Software/PulseAudio/>
- ALSA: <https://www.alsa-project.org/>
