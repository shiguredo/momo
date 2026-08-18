# Ubuntu 環境のカメラで MJPEG より YUV が優先されるのを修正する

- Created: 2025-09-17
- Completed: 2025-09-17
- Branch: feature/fix-ubuntu-camera-mjpeg-priority
- Polished: {YYYY-MM-DD}

## 目的

VGA より大きい解像度では MJPEG を優先する、という既存方針どおりにフォーマット選択が動くようにする。YUV が先に採用されると USB 帯域を圧迫し、カメラによっては期待した解像度・フレームレートが出ない。

## 現状

`sora::V4L2VideoCapturer::StartCapture` は、指定が無ければ VGA より大きいとき MJPEG を先頭にした候補配列 `fmts` を使う。しかし候補を走査する二重ループが、最初に一致したフォーマットで止まらず最後まで回り、後から一致した YUV 系で `found_format` を上書きしていた。

## 設計方針

`found_format` が決まったら外側ループを抜ける。候補配列の先頭（MJPEG 優先）を採用する。`--force-i420` / `--force-yuy2` / `--force-nv12` / `use_native` の専用経路は候補が 1 系統なので影響しない。

## 完了条件

- VGA より大きい解像度で、カメラが MJPEG をサポートしていれば MJPEG が選ばれる
- `force_*` 指定時の挙動は変わらない

## 解決方法

`src/sora-cpp-sdk/src/v4l2/v4l2_video_capturer.cpp` の `V4L2VideoCapturer::StartCapture` で、フォーマット探索の外側ループ条件に `!found_format` を追加した。最初に一致した `fmts[i]` で探索を終える。
