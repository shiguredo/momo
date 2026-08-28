# V4L2Runner が `POLLERR` / `POLLHUP` を無視し破棄時にハングする

- Created: 2026-08-28
- Completed: {YYYY-MM-DD}
- Branch: feature/fix-v4l2-runner-pollerr-hang
- Polished: {YYYY-MM-DD}

## 目的

`V4L2Runner` の poll ループが `POLLIN` / `POLLPRI` しか見ない。デバイスエラーや切断で `POLLERR` / `POLLHUP` が立ってもループが続き、デストラクタの `thread_.Finalize()` がスレッド join でハングしうる。これを修正する。

## 現状

- `src/sora-cpp-sdk/src/hwenc_v4l2/v4l2_runner.cpp` の `PollProcess()` が `pollfd` を `{fd_, POLLIN | POLLPRI, 0}` で待つ
- `revents` の `POLLERR` / `POLLHUP` / `POLLNVAL` を見ていない
- 脱出条件は `abort_poll_ && output_buffers_available_.size() == src_count_`、または `poll` が `-1`
- `~V4L2Runner()` は `abort_poll_ = true` のあと `thread_.Finalize()` する。バッファが全部戻らないと `abort_poll_` だけでは抜けない
- デバイスが壊れると `VIDIOC_DQBUF` も失敗し続け、利用可能インデックスが揃わない

## 設計方針

- `poll` 後に `POLLERR` / `POLLHUP` / `POLLNVAL` を検出したらログを出してループを抜ける
- エラー時は `abort_poll_` を立て、join がバッファ回収待ちで止まらないようにする
- 正常な `POLLIN` / `POLLPRI` 処理は維持する
- 修正は momo と sora-cpp-sdk の両方に入れ、`update-last-updated.sh` で同期する

## 完了条件

- デバイスエラー / hangup で poll スレッドが終了する
- デストラクタが join でハングしない
- 通常のエンコード / デコード完了待ちは従来通り動作する

## 解決方法

未着手 (PR 作成後に追記する)
