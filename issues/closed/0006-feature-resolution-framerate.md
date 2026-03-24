# --resolution / --framerate の実装

## 概要

CLI オプションはパースされるが、実処理に渡されておらず 640x480 / 30fps 固定。

## 現状

- `src/main.rs` で `_resolution`, `_framerate` に束縛されるだけで CommonConfig に含まれない
- 映像キャプチャは 640x480 / 30fps 固定

## 必要な実装

- `--resolution`: QVGA (320x240) / VGA (640x480) / HD (1280x720) / FHD (1920x1080) / 4K (3840x2160) / 任意 WxH のパースと適用
- `--framerate`: 1-120fps の範囲指定と適用
- CommonConfig に追加し、P2P / Ayame の映像キャプチャに渡す

## 解決方法

- `parse_resolution()` 関数を追加: QVGA/VGA/HD/FHD/4K プリセットと WxH 形式をパース
- `CommonConfig` に `video_width`, `video_height`, `framerate` を追加
- P2P / Ayame / Sora の各 Config 構造体にも追加し、ハードコード値 (640x480/30fps) を置換
- `fake.rs` の `start_fake_video_thread` にも `fps` 引数を追加

Completed: 2026-03-22
