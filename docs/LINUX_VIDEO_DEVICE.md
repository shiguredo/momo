# Linux でのビデオデバイスアクセス

## 概要

Linux 環境で momo を使用してビデオデバイス（カメラ）にアクセスするには、適切なユーザー権限が必要です。

momo はデバイスファイルではなくデバイス名を指定してカメラを選択します。

## ユーザー権限の設定

Linux では、ビデオデバイスへのアクセスにグループ権限が必要です。

### 必要なグループ

- `video`: ビデオデバイス（`/dev/video*` など）へのアクセスに必要

### グループへの参加

```bash
sudo usermod -a -G video $USER
```

### 権限の確認

デバイスファイルの権限を確認:

```bash
ls -l /dev/video*
```

典型的な出力例:

```console
crw-rw---- 1 root video 81, 0 Oct 15 10:00 /dev/video0
crw-rw---- 1 root video 81, 1 Oct 15 10:00 /dev/video1
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
   newgrp video
   ```

3. 一時的にグループ権限で実行:

   ```bash
   sg video -c 'コマンド'
   ```

## ビデオデバイス一覧の確認

### momo でのデバイス確認

```bash
./_build/ubuntu-24.04_x86_64/release/momo/momo --list-devices
```

正常に動作している場合の出力例:

```
=== Available video devices ===

  [/dev/video0] Insta360 Link: Insta360 Link (usb-0000:05:00.4-2.4):
    Supported formats:
      [0] MJPG (Motion-JPEG)
        1920x1080 (30 fps, 25 fps, 24 fps)
        1920x1440 (30 fps, 25 fps, 24 fps)
        1280x720 (30 fps, 25 fps, 24 fps)
        1280x960 (30 fps, 25 fps, 24 fps)
        3840x2160 (30 fps, 25 fps, 24 fps)
      [1] H264 (H.264)
        3840x2160 (30 fps, 25 fps, 24 fps)
        1920x1080 (30 fps, 25 fps, 24 fps)
        1920x1440 (30 fps, 25 fps, 24 fps)
        1280x720 (30 fps, 25 fps, 24 fps)
        1280x960 (30 fps, 25 fps, 24 fps)

  [/dev/video2] Logitech StreamCam (usb-0000:05:00.3-1):
    Supported formats:
      [0] YUYV (YUYV 4:2:2)
        640x480 (30 fps, 24 fps, 20 fps, 15 fps, 10 fps, 15/2 fps, 5 fps)
        176x144 (30 fps, 24 fps, 20 fps, 15 fps, 10 fps, 15/2 fps, 5 fps)
        320x240 (30 fps, 24 fps, 20 fps, 15 fps, 10 fps, 15/2 fps, 5 fps)
        ...
```

### v4l2-ctl でのデバイス確認

```bash
v4l2-ctl --list-devices
```

### カーネルが認識しているビデオデバイス

```bash
ls /dev/video*
```

## ビデオデバイスの選択

### デバイスの指定方法

ビデオデバイスを指定できます:

```bash
./_build/ubuntu-24.04_x86_64/release/momo/momo --video-input-device "Logitech StreamCam" p2p
```

デバイスは以下の方法で指定できます:

- デバイス名: `--video-input-device "Logitech StreamCam"`
- Bus info: `--video-input-device "usb-0000:05:00.3-1"`

### 同一カメラデバイスを複数接続している場合

同一カメラデバイスを複数接続している場合、デバイス名が同じであるため、個別に指定することができません。
その場合は、デバイス名を指定する代わりに、`Bus info` を指定することができます。

`--list-devices` オプションで表示される `Bus info`（括弧内の `usb-...` の部分）を使って指定できます。

```bash
./_build/ubuntu-24.04_x86_64/release/momo/momo --video-input-device "usb-0000:05:00.4-2.4" p2p
```

## トラブルシューティング

### ビデオデバイスが表示されない

**原因と対処:**

1. ユーザーが `video` グループに所属していない

   ```bash
   sudo usermod -a -G video $USER
   # ログアウトして再ログイン、または newgrp video
   ```

2. カメラが接続されていない、または認識されていない

   ```bash
   ls /dev/video*
   v4l2-ctl --list-devices
   ```

3. カーネルドライバの問題

   ```bash
   dmesg | grep -i video
   ```

### デバイスファイルのアクセス権を確認

```bash
ls -l /dev/video*
```

## 参考情報

- Video4Linux (V4L2): <https://www.kernel.org/doc/html/latest/userspace-api/media/v4l/v4l2.html>
