# 死にコードとコメントアウトされた古いコードを削除する

- Created: 2026-08-19
- Completed: {YYYY-MM-DD}
- Branch: feature/remove-dead-code
- Polished: 2026-09-02
- Milestone: 2026.1.0

## 目的

コードベースに死にコード (呼び出されない関数・ビルド対象なのに未使用のファイル) と、コメントアウトされた古い実行コード・デバッグ残骸が残っている。「壊れた窓」を放置せず、正式リリース前に削除する。説明コメントの日本語化は `0037-refactor-japanese-comments` の担当であり、本 issue では行わない。

## 現状 (削除対象)

行番号は書かない。着手時にシンボルと前後の説明コメントで特定する。

### 完全な死にコード (本 issue で削除する)

- `Util::GenerateRandomNumericChars` (`src/util.cpp` / `src/util.h`) — リポジトリ内の呼び出し 0。`Util::GenerateRandomChars` は使用中なので残す
- `src/rtc/native_buffer.cpp` / `native_buffer.h` — 生きた参照は定義側のみ。`screen_video_capturer.cpp` ではコメントアウト内だけ。`CMakeLists.txt` の `momo` ソースに含まれている。`v4l2_native_buffer` は別物で対象外
- `src/sora-cpp-sdk/src/hwenc_v4l2/v4l2_video_codec.cpp` と `include/sora/hwenc_v4l2/v4l2_video_codec.h` — CMake 非掲載。両方とも存在しない `sora/sora_video_codec.h` を include し、`GetV4L2VideoCodecCapability` の宣言・定義だけがある
- `nvcodec_video_encoder_cuda.cpp` の `ShowEncoderCapability` — 関数本体は有効な定義 (`exit(1)` を含む)。コメントアウトなのは `NvCodecVideoEncoderCudaImpl` コンストラクタ内の呼び出し 1 行だけ。呼び出しも関数定義も削除する
- `jetson_video_encoder.cpp` の `H264HWENC_HEADER_DEBUG` / `hex_dump` / `save_to_file` — 定義は生きているが、参照はコメントアウト内（またはマクロ未使用）のみ。`hwenc_jetson` は momo 独自で上流同期の対象外
- `src/fix_cuda_noinline_macro_error.h` — momo の `src/` 直下の複製で参照 0。実際の include は `sora/fix_cuda_noinline_macro_error.h`

### コメントアウトされた実行コード (本 issue で削除する)

対象はコメントアウトされた **実行コード** に限る。前後の説明コメントは残す (0037 の翻訳対象になりうる)。

- `screen_video_capturer.cpp` — デバッグログと古い `NativeBuffer` 経路など、実行コードのコメントアウト塊。著作権ブロックと対策の説明コメントは残す
- `vpl_video_encoder.cpp` — デバッグ用マクロや無効な実行行。幅が 16 の倍数である旨などの説明コメントは残す
- `SoraClient` の encodings 変換ループ内 — `webrtc::RtpEncodingParameters` のフィールド宣言を写したコメントアウト群 (`ssrc` / `bitrate_priority` / `rid` 等)。直前の `InitTracks` と `simulcast` 分岐の実行コードは消さない
- `momo_video_encoder_factory.cpp` — H.265 Software 時に `return nullptr` するコメントアウト 3 行
- `rtc_manager.cpp` — `CreateModularPeerConnectionFactory` のコメントアウト 2 行。前後の日本語説明は残す
- `aligned_encoder_adapter.cpp` — `RTC_LOG` のコメントアウト塊。クロップ計算の説明コメントは残す
- `Websocket::CreateSSLContext` 内の `//ctx.set_default_verify_paths();`。`NormalizeHostForVerification` の説明コメントは別物で残す

### 未使用のヘッダインクルード (本 issue で削除する)

- `screen_video_capturer.cpp` の `<iostream>` / `native_buffer.h`、`screen_video_capturer.h` の `video_capture.h`
- `watchdog.cpp` / `peer_connection_observer.cpp` の `<iostream>`
- `serial_data_manager.cpp` の `boost/asio/post.hpp` 重複 (片方だけ残す)。`std::cerr` 用の `<iostream>` は使用中なので残す

### 削除しない (参照はあるが「死にコード」と誤認しやすいもの)

- `ScalableVideoTrackSource::Config::on_frame` — momo 側に代入は無いが、`scalable_track_source.cpp` がコールバックとして読む公開メンバ。削除は SDK の公開 API 変更になる
- `JetsonJpegDecoderPool::Push` — キューへの `push` はコメントアウトされているが、`JetsonJpegDecoder` デストラクタが呼ぶ。関数ごと消すとビルドが壊れる。無効化したキュー (`decoder_queue_` / `Pop` 内のコメントアウト塊) を整理するなら、ヘッダのメンバ、`Push` の本体、デストラクタ呼び出しをセットで直す。JetPack 5.1.2 / momo PR 297 の理由コメントは残す

### 本 issue の完了対象外 (現状の把握用。別判断が必要)

closed `0004-change-jetson-multistrap-to-sysroot` は、`buildbase.py` の `install_rootfs()` をリポジトリから削除せず、`run.py` の呼び出しだけを外すと決めた。`install_sdl2` / `install_grpc` も同テンプレート由来。本 issue では削除しない。

CI ワークフローの死設定と意図的無効化は `0056-remove-ci-dead-workflow-settings` の担当である。

## 設計方針

- コメントアウトされた実行コードは削除する。説明コメントは翻訳も削除もしない (0037)
- 各対象は grep で参照 0 を再確認してから消す。生きている制御フロー・説明コメントを行番号で消さない
- `src/sora-cpp-sdk/` のうち `hwenc_jetson` 以外は、同期元 2026.2.1 (`LAST_UPDATED_SORA_CPP_SDK`) で既に無いかを確認してから消す。残る差分は次回上流同期で上書きされうる
- `hwenc_jetson` は momo 独自なので、参照確認だけで消してよい
- `buildbase.py` は 0004 の方針を維持する
- CI ワークフローは `0056-remove-ci-dead-workflow-settings` で扱う。本 issue では触らない

## 完了条件

- 「完全な死にコード」「コメントアウトされた実行コード」「未使用のヘッダインクルード」に列挙した対象が削除されている
- 「削除しない」に列挙したシンボルを、呼び出し側を直さずに関数・メンバごと消していない
- `buildbase.py` と `.github/workflows/` を本 issue の差分に含めていない
- 対象を含む構成でビルドが通る (momo 固有ファイルは全構成、`sora-cpp-sdk` 側は該当コーデックを含む構成)
- 既存の E2E が通る

## 解決方法

未着手 (PR 作成後に追記する)
