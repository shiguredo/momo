# libcamera キャプチャ停止時に保持中フレームのリリースコールバックが破棄済み request を参照する

- Created: 2026-08-19
- Completed: {YYYY-MM-DD}
- Branch: feature/fix-libcamera-use-after-free
- Polished: 2026-08-20
- Milestone: 2026.1.0

## 目的

Raspberry Pi の libcamera キャプチャ (`LibcameraCapturer`) が、`--use-libcamera-native` かつ非サイマルキャストの native 出力時に、停止 (`StopCapture`) で request オブジェクトを破棄した後、WebRTC 側が保持しているフレームのリリースコールバックが破棄済み request の `queueRequest()` を呼び、`libcamerac_Request_buffers()` が破棄済みメモリを deref して use-after-free (クラッシュ) する可能性がある。これを修正する。

## 現状

- `src/sora-cpp-sdk/src/hwenc_v4l2/libcamera_capturer.cpp` の `StopCapture()` が `requests_.clear()` を実行し、`std::vector<std::shared_ptr<libcamerac_Request>>` である `requests_` の最後の参照を破棄する
- 一方、`requestComplete()` が native 出力時に `V4L2NativeBuffer` のリリースコールバックへ raw ポインタ `request` と `this` をキャプチャして渡す (`[this, request]() { queueRequest(request); }`)
- `V4L2NativeBuffer` は最後の参照破棄時にこのコールバックを呼ぶため、WebRTC がフレームを保持したまま停止すると、最終リリース時に破棄済み request と `this` を deref する
- `queueRequest()` には `camera_started_` による停止ガードがあるが、`libcamerac_Request_buffers()` / `libcamerac_Request_reuse()` の deref より後に置かれており、破棄済み request への deref を防げない。また `requests_.clear()` は `camera_stop_mutex_` のロック外で実行される
- 変更対象は sora-cpp-sdk から vendoring されたファイルであり、0013 (sora-cpp-sdk 2026.2.1 同期) と並走する。上流 2026.2.1 でも本バグは未修正であることを確認済み。0013 は 0023 を同期確認対象から除外しているため、本修正を先に適用すると 0013 の同期で上書きされても検出されない

## 設計方針

- リリースコールバックがカメラ停止後・オブジェクト破棄後に呼ばれても安全になるようにする
- request の生存期間をフレームのリリース完了まで共有所有 (shared_ptr) で延長し、raw ポインタのキャプチャをやめる
- `this` も同様に破棄後にコールバックが呼ばれうるため、コールバックが参照する状態をオブジェクト本体の生存期間から独立させる (共有所有への分離等)
- 既存の `camera_started_` ガードを deref より前に移動し、停止後は `queueRequest()` が何もせず戻るようにする
- 0013 の同期を先に完了させてから本修正を適用する。0013 の同期完了後に本バグが未修正のままであることを再確認し、上流で解消済みの場合は修正せず pending にする。修正する場合は momo 側に加えて sora-cpp-sdk 側にも反映し、`update-last-updated.sh` で同期する

## 完了条件

- libcamera キャプチャ停止中・停止直後に use-after-free が発生しない (ASAN ビルドで再現確認する)
- 通常のキャプチャ・停止が従来通り動作する
- 停止時に保持中フレームがあっても安全

## 解決方法

未着手 (PR 作成後に追記する)
