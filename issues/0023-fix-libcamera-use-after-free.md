# libcamera キャプチャ停止時に保持中フレームのリリースコールバックが破棄済み request を参照する

- Created: 2026-08-19
- Completed: {YYYY-MM-DD}
- Branch: feature/fix-libcamera-use-after-free
- Polished: {YYYY-MM-DD}

## 目的

Raspberry Pi の libcamera キャプチャ (`LibcameraCapturer`) が、停止 (`StopCapture`) 時に `requests_.clear()` で request オブジェクトを破棄した後、WebRTC 側が保持しているフレームのリリースコールバックが破棄済み request の `queueRequest()` を呼び、`libcamerac_Request_buffers()` が破棄済みメモリを deref して use-after-free (クラッシュ) する可能性がある。これを修正する。

## 現状

- `src/sora-cpp-sdk/src/hwenc_v4l2/libcamera_capturer.cpp` の `StopCapture()` (332 行) が `requests_.clear()`
- 一方、`V4L2NativeBuffer` のリリース時に `queueRequest(request)` (406 行) が raw ポインタ `request` をキャプチャして呼ぶ (424-457 行)
- WebRTC がフレームを保持したまま停止すると、最終リリース時に破棄済み request を deref する

## 設計方針

- request の生存期間を、保持中フレームのリリース完了まで延長する (shared_ptr 等)
- または停止時に、保留中のフレームのリリースを待ってから request を破棄する
- 停止後は `queueRequest()` が呼ばれても安全に無視されるようにする

## 完了条件

- libcamera キャプチャ停止中・停止直後に use-after-free が発生しない
- 通常のキャプチャ・停止が従来通り動作する
- 停止時に保持中フレームがあっても安全

## 解決方法

未着手 (PR 作成後に追記する)
