# 死にコードとコメントアウトされた古いコードを削除する

- Created: 2026-08-19
- Completed: {YYYY-MM-DD}
- Branch: feature/remove-dead-code
- Polished: {YYYY-MM-DD}

## 目的

コードベースに死にコード (呼び出されない関数・ビルド対象なのに未使用のファイル) と、コメントアウトされた古いコード・デバッグ残骸が多数残っている。「壊れた窓」を放置せず、正式リリース前に削除する。

## 現状 (削除対象)

### 完全な死にコード

- `src/util.cpp` / `src/util.h` の `Util::GenerateRandomNumericChars()` — 呼び出し 0、`rand()` 使用
- `src/rtc/native_buffer.cpp` / `native_buffer.h` — 参照は `screen_video_capturer.cpp` のコメントアウトのみ。`CMakeLists.txt` のビルド対象
- `src/sora-cpp-sdk/src/hwenc_v4l2/v4l2_video_codec.cpp` — ビルド対象外、存在しないヘッダ `sora/sora_video_codec.h` を include
- `src/sora-cpp-sdk/src/hwenc_nvcodec/nvcodec_video_encoder_cuda.cpp` の `ShowEncoderCapability()` — コメントアウトされた約 95 行、`exit(1)` を含む
- `src/sora-cpp-sdk/src/hwenc_jetson/jetson_video_encoder.cpp` の `H264HWENC_HEADER_DEBUG` / `hex_dump` / `save_to_file` — コメントアウト内のみで使用
- `src/sora-cpp-sdk/include/sora/scalable_track_source.h` の `on_frame` — 設定箇所 0
- `src/sora-cpp-sdk/src/hwenc_jetson/jetson_jpeg_decoder_pool.cpp` の無効化されたキュー実装 — `Push` が no-op
- `buildbase.py` の `install_rootfs()` — multistrap 廃止済みなのに残存。他にもテンプレート由来の未使用関数多数 (`install_sdl2` / `install_grpc` 等)
- `src/fix_cuda_noinline_macro_error.h` — どこからも参照されない (実際に使われるのは sora-cpp-sdk 側)

### コメントアウトされた古いコード

- `src/rtc/screen_video_capturer.cpp` — コメントアウト 12 箇所 (デバッグログ・古い実装残骸)
- `src/sora-cpp-sdk/src/hwenc_vpl/vpl_video_encoder.cpp` — コメントアウト多数 (デバッグ用マクロ等)
- `src/sora/sora_client.cpp` (469-480 行) — `RtpEncodingParameters` のフィールド一覧コメント
- `src/rtc/momo_video_encoder_factory.cpp` / `rtc_manager.cpp` / `aligned_encoder_adapter.cpp` のコメントアウト
- `src/websocket.cpp` (26 行) — `//ctx.set_default_verify_paths();`

### 未使用のヘッダインクルード

- `src/rtc/screen_video_capturer.cpp` の `<iostream>` / `native_buffer.h`、`screen_video_capturer.h` の `video_capture.h`
- `src/watchdog.cpp` / `peer_connection_observer.cpp` の `<iostream>`
- `src/serial_data_channel/serial_data_manager.cpp` の `boost/asio/post.hpp` 重複インクルード

### CI のコメントアウト・死に設定

- `.github/workflows/build.yml` (15 行) — 存在しない `claude.yml` の paths-ignore
- `.github/workflows/build.yml` (284-286 行) — コメントアウトされた Jetson の download step
- `.github/workflows/e2e-test.yml` (18-22, 42-44 行) — コメントアウトされた schedule / Windows matrix

## 設計方針

- `buildbase.py` はテンプレート由来 (上流 melpon/buildbase と同期運用) のため、削除する場合は上流との差分管理方針を確認してから行う
- `src/sora-cpp-sdk/` の削除候補は同期元 (2026.2.1) で削除済みかを確認してから行う
- 各削除対象について、実際に参照が無いことを確認してから削除する

## 完了条件

- 上記の死にコード・コメントアウト残骸が削除されている
- 全プラットフォームでビルドが通る
- E2E テストが通る

## 解決方法

未着手 (PR 作成後に追記する)
