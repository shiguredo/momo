# `--video-device` を `--video-input-device` に変更する

- Created: 2025-10-16
- Completed: 2025-10-16
- Branch: feature/video-input-device
- Polished: {YYYY-MM-DD}

## 目的

オーディオ入力デバイスオプション (`--audio-input-device`) と命名を揃え、ビデオ入力デバイスであることが名前から分かるようにする。`--no-video-device` も同様に `--no-video-input-device` へ変更する。

## 現状

CLI は `src/util.cpp` の `Util::ParseArgs` で `--video-device` と `--no-video-device` を受け付け、`MomoArgs::video_device` / `MomoArgs::no_video_device` に格納していた。オーディオ側は後から `--audio-input-device` を追加する予定があり、ビデオ側だけ `device` だと入出力の区別が付かない。

## 設計方針

オプション名だけを変更し、内部フィールド名 (`video_device` / `no_video_device`) や選択ロジックは維持する。旧名のエイリアスは残さない。破壊的変更として `CHANGES.md` に `[CHANGE]` で記録する。

## 完了条件

- `--video-input-device` と `--no-video-input-device` が利用できる
- `--video-device` と `--no-video-device` は受け付けない
- ドキュメントと E2E テストの起動引数が新名に追従している

## 解決方法

PR #435 で以下を変更した。

- `src/util.cpp` の CLI11 登録を `--video-input-device` / `--no-video-input-device` に変更する
- `test/momo.py` の起動引数を新名に合わせる
- `doc/USE.md`、`doc/SETUP_MAC.md`、`doc/LINUX_VIDEO_DEVICE.md` の記述を新名に合わせる
