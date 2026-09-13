# libcamera キャプチャが `mmap` の失敗 (`MAP_FAILED`) を検査しない

- Created: 2026-08-28
- Completed: {YYYY-MM-DD}
- Branch: feature/fix-libcamera-mmap-failed
- Polished: 2026-09-13

## 目的

`LibcameraCapturer` がフレームバッファを `mmap` したあと戻り値が `MAP_FAILED` かを見ない。失敗したポインタを `Span` に入れて後段が deref し、クラッシュする。これを修正する。

## 現状

- `src/sora-cpp-sdk/src/hwenc_v4l2/libcamera_capturer.cpp` のバッファ確保が `mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0)` し、結果を `static_cast<uint8_t*>(memory)` して `mapped_buffers_` に入れる
- `MAP_FAILED` (`(void*)-1`) の分岐が無い
- 同じリポジトリの `src/sora-cpp-sdk/src/v4l2/v4l2_video_capturer.cpp` と `src/sora-cpp-sdk/src/hwenc_v4l2/v4l2_buffers.cpp` は `MAP_FAILED` を検査している
- `native_frame_output` のときは mmap せず fd だけ保持する。対象は mmap する経路である
- `0023` は停止時の request UAF であり、本指摘は mmap 失敗である

## 設計方針

- `mmap` 直後に戻り値が `MAP_FAILED` なら英語のエラーログを出し、失敗したエントリを `mapped_buffers_` に入れず `StartCapture()` を `-1` で失敗させる
- 途中まで成功した mmap の掃除は `StartCapture()` 内で明示的に `munmap` せず、`ReleaseLibcamera()` に任せる。`StartCapture()` が `-1` を返すと `Create()` が `nullptr` を返してインスタンスが破棄され、`ReleaseLibcamera()` が `mapped_buffers_` を走査して `munmap` する。明示的に `munmap` 後にエントリを残すと二重 `munmap` になるため、どちらか一方に責務を固定する
- 修正は momo と sora-cpp-sdk の両方に入れ、`update-last-updated.sh` で同期する

## 完了条件

- `mmap` 失敗で `MAP_FAILED` をフレームデータとして使わない
- 成功時は従来通りキャプチャできる
- 失敗時に英語のエラーログが出て開始が失敗する
- 失敗時は途中まで確保した mmap が `ReleaseLibcamera()` で一度だけ解放され、漏れも二重解放もない

## 解決方法

未着手 (PR 作成後に追記する)
