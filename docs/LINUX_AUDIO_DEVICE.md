# Linux での音声デバイスアクセス

## 概要

Linux 環境で momo を使用して音声デバイスにアクセスするには、PipeWire と適切なユーザー権限が必要です。

momo は PulseAudio API 経由で音声デバイスにアクセスします。最近の Ubuntu 環境では PipeWire が PulseAudio 互換レイヤー（pipewire-pulse）を提供しているため、PipeWire 経由で動作します。

## 必要なソフトウェア

### PipeWire のインストール

```bash
sudo apt install pipewire pipewire-pulse wireplumber
```

Ubuntu 24.04 以降では PipeWire がデフォルトでインストールされています。

### PipeWire の起動

systemd 経由で起動する場合:

```bash
systemctl --user start pipewire pipewire-pulse wireplumber
```

### インストール確認

PipeWire と pipewire-pulse がインストールされているか確認:

```bash
systemctl --user status pipewire pipewire-pulse wireplumber
```

正常に動作している場合、3 つのサービスすべてが `Active: active (running)` と表示されます:

```
● pipewire.service - PipeWire Multimedia Service
     Active: active (running) since ...

● pipewire-pulse.service - PipeWire PulseAudio
     Active: active (running) since ...

● wireplumber.service - Multimedia Service Session Manager
     Active: active (running) since ...
```

pipewire-pulse が動作していない場合、momo は音声デバイスにアクセスできません。

## ユーザー権限の設定

Linux では、音声デバイスへのアクセスにグループ権限が必要です。

### 必要なグループ

- `audio`: 音声デバイス（`/dev/snd/*` など）へのアクセスに必要

### グループへの参加

```bash
sudo usermod -a -G audio $USER
```

### 権限の確認

デバイスファイルの権限を確認:

```bash
ls -l /dev/snd/
```

典型的な出力例:

```
crw-rw---- 1 root audio 116, 0 Oct 15 10:00 /dev/snd/controlC0
```

この例では、`audio` グループに所属するユーザーのみが読み書き可能です。

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
   ```

3. 一時的にグループ権限で実行:

   ```bash
   sg audio -c 'コマンド'
   ```

## 音声デバイス一覧の確認

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
```

### pactl でのデバイス確認

音声入力デバイス:

```bash
pactl list sources short
```

音声出力デバイス:

```bash
pactl list sinks short
```

### カーネルが認識しているサウンドカード

```bash
cat /proc/asound/cards
```

### PipeWire の状態確認

```bash
pw-cli ls Node
```

## 音声デバイスの選択

### デバイスの指定方法

音声入力デバイスと音声出力デバイスを指定できます:

```bash
./momo --audio-input-device 1 --audio-output-device 0 ...
```

デバイスは以下の方法で指定できます:

- インデックス番号: `--audio-input-device 1`
- デバイス名: `--audio-input-device "HyperX QuadCast S Analog Stereo"`

## トラブルシューティング

### 音声デバイスが表示されない

**症状:**

```
Warning: AudioDeviceModule::Init failed (error code: -1). Audio device enumeration is not available.
Note: This may occur if PulseAudio or ALSA is not running.
```

**原因と対処:**

1. PipeWire または pipewire-pulse が起動していない

   ```bash
   systemctl --user start pipewire pipewire-pulse wireplumber
   ```

2. ユーザーが `audio` グループに所属していない

   ```bash
   sudo usermod -a -G audio $USER
   # ログアウトして再ログイン、または newgrp audio
   ```

3. PipeWire がハードウェアを認識していない

   ```bash
   systemctl --user restart pipewire pipewire-pulse wireplumber
   ```

### Dummy デバイスしか表示されない

**原因:**

- PipeWire がハードウェアデバイスを認識していない
- ユーザー権限の問題

**対処:**

1. グループ権限を確認:

   ```bash
   groups
   # audio が含まれているか確認
   ```

2. PipeWire を適切な権限で再起動:

   ```bash
   sg audio -c 'systemctl --user restart pipewire pipewire-pulse wireplumber'
   ```

3. デバイスが認識されているか確認:

   ```bash
   cat /proc/asound/cards
   pw-cli ls Node
   ```

### 音声デバイスが動作しない

**対処:**

1. PipeWire のログを確認:

   ```bash
   journalctl --user -u pipewire -f
   journalctl --user -u pipewire-pulse -f
   ```

2. pactl でデバイスの状態を確認:

   ```bash
   pactl list sources
   pactl list sinks
   ```

3. デバイスファイルのアクセス権を確認:

   ```bash
   ls -l /dev/snd/
   ```

## 参考情報

- PipeWire: <https://pipewire.org/>
- WirePlumber: <https://pipewire.pages.freedesktop.org/wireplumber/>
- PulseAudio API: <https://www.freedesktop.org/wiki/Software/PulseAudio/>
