# Raspberry Pi 用の E2E テストを追加する

- Created: 2025-09-19
- Completed: 2025-09-27
- Branch: feature/e2e-test-raspberry-pi
- Polished: {YYYY-MM-DD}

## 目的

Raspberry Pi OS armv8 上で、libcamera キャプチャーと V4L2 M2M H.264 エンコーダを含む Sora モードの経路を CI で継続的に確認する。

## 現状

E2E は macOS / Ubuntu x86_64 などの fake capture 中心で、Raspberry Pi の libcamera と V4L2 M2M は手動確認に寄っていた。実機経路の回帰を CI で止められなかった。

## 設計方針

- GitHub Actions の self-hosted runner で `raspberry-pi-os_armv8` を回す
- 環境変数 `RASPBERRY_PI` と `TEST_SORA_MODE_SIGNALING_URLS` が無いときはスキップする
- 実カメラは `--use-libcamera`、音声デバイスは使わない
- VP8 / VP9 / AV1 はソフトウェア、H.264 は `V4L2M2M H264` を期待する

## 完了条件

- `test/test_sora_mode_raspberry_pi.py` がある
- `e2e-test.yml` の matrix に `raspberry-pi-os_armv8` がある
- libcamera + V4L2 M2M の sendonly 接続が統計情報で確認できる

## 解決方法

PR #424 でテストを追加し、PR #426 で self-hosted 上の不安定さを潰した。

- `test/test_sora_mode_raspberry_pi.py` を追加する。`Momo` に `use_libcamera=True`、`no_audio_device=True`、`fake_capture_device=False` を渡す
- `.github/workflows/e2e-test.yml` に `raspberry-pi-os_armv8` ジョブを追加し、`RASPBERRY_PI=true` を渡して `test_sora_mode_raspberry_pi.py` を実行する
- 接続待ちが足りない場合は `initial_wait` を延ばす（後続で 20 秒まで延長）
