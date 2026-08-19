# Jetson デコーダが解像度変更時の失敗で自己スレッドを Join してデッドロックする

- Created: 2026-08-19
- Completed: {YYYY-MM-DD}
- Branch: feature/fix-jetson-decoder-self-join-deadlock
- Polished: {YYYY-MM-DD}

## 目的

Jetson ハードウェアデコーダ (`JetsonVideoDecoder`) の CaptureLoop スレッド内で `SetCapture()` が呼ばれ、その中の `INIT_ERROR` マクロが `Release()` → `JetsonRelease()` → `capture_loop_.Finalize()` を呼ぶ。`Finalize()` は自分自身のスレッドを Join するため、解像度変更イベント後の ioctl 失敗 (ハードウェア異常・デバイス切断) で永久にデッドロックする。これを修正する。

## 現状

- `src/sora-cpp-sdk/src/hwenc_jetson/jetson_video_decoder.cpp` の `CaptureLoop()` が `SetCapture()` を CaptureLoop スレッドから呼ぶ
- `SetCapture()` 内の `INIT_ERROR` マクロ (34-39 行) が `Release()` → `JetsonRelease()` → `capture_loop_.Finalize()` (206 行) を呼び、自分自身を Join する
- `SetCapture()` は CaptureLoop スレッドからのみ呼ばれる (265, 271 行)

## 設計方針

- CaptureLoop スレッド内で `Release()` が呼ばれないようにする
  - `SetCapture()` の失敗時にスレッド内から `Release()` を呼ばず、エラーフラグ (`got_error_`) を立てて CaptureLoop を終了させ、外部 (デコーダの `Release()` 呼び出し側) に破棄を委ねる
  - `capture_loop_.Finalize()` をスレッド外から呼ぶようにする
- 解像度変更失敗時のエラーハンドリングを明示する

## 完了条件

- 解像度変更イベント後の ioctl 失敗でデッドロックしない
- 正常時の解像度変更は従来通り動作する
- エラー時にデコーダが破棄され、クラッシュやハングが無い

## 解決方法

未着手 (PR 作成後に追記する)
