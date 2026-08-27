# --no-video-input-device と --fake-capture-device の併用で nullptr メンバ呼び出しによりクラッシュする

- Created: 2026-08-19
- Completed: 2026-08-27
- Branch: feature/fix-fake-capture-no-video-crash
- Polished: 2026-08-19
- Milestone: 2026.1.0

## 目的

`--no-video-input-device` (CLI 名、内部フィールド `MomoArgs::no_video_device`) と `--fake-capture-device` を同時に指定した場合、`src/main.cpp` の `create_adm` ラムダ内で `capturer` が `nullptr` のまま `static_cast<FakeVideoCapturer*>(capturer.get())->SetAudioCapturer()` を呼び、確実にクラッシュする (`USE_FAKE_CAPTURE_DEVICE=ON` の `macOS` / `ubuntu-22.04_x86_64` / `ubuntu-24.04_x86_64` のみで発現する)。これを修正する。

## 現状

- `src/main.cpp` の `create_adm` ラムダ (`src/main.cpp:331` の `if (args.fake_capture_device && !args.no_audio_device)` 分岐) 内で `capturer` を null チェックせず `static_cast<FakeVideoCapturer*>(capturer.get())->SetAudioCapturer(fake_audio_capturer)` (`src/main.cpp:339`) を呼ぶ
- `capturer` は `src/main.cpp` のビデオキャプチャ生成 IIFE (`src/main.cpp:191` の `if (args.no_video_device) return nullptr;`) で `args.no_video_device == true` なら決定的に `nullptr` になる。`FakeVideoCapturer::Create` より前に `return` するため、`USE_FAKE_CAPTURE_DEVICE` 定義ビルド時のみ再現する
- `create_adm` は `src/rtc/rtc_manager.cpp` の `worker_thread_->BlockingCall` (`src/rtc/rtc_manager.cpp:155`) で `config_.create_adm` が設定されていれば同期的に必ず呼ばれ、起動直後にクラッシュが再現する。`capturer` は値キャプチャで保持されるため寿命は保証されるが `nullptr` deref は防げない
- 真理値表では `no_video_device == true && fake_capture_device == true && no_audio_device == false` のときのみクラッシュし、`no_audio_device == true` なら `create_adm` 未設定で回避される

## 設計方針

- CLI バリデーションで併用を禁止し、二重防御として `create_adm` 内でもガードする
  - `src/main.cpp` の `capturer` 生成直後 (`src/main.cpp:275` 直後) に `#if defined(USE_FAKE_CAPTURE_DEVICE)` で囲んで `if (args.no_video_device && args.fake_capture_device) { std::cerr << "error: --fake-capture-device cannot be used with --no-video-input-device" << std::endl; return 2; }` で禁止する (`src/main.cpp:184` の `hw_mjpeg_decoder` チェックと同形式、ログは英語)。`--help` の help 文字列に `Cannot be used with --no-video-input-device` を追記する
  - 二重防御として `src/main.cpp:336` ラムダ内で `if (!capturer) { RTC_LOG(LS_WARNING) << "CreateADM: capturer is null, skip SetAudioCapturer"; return fake_audio_capturer; }` を追加する。コメントは日本語、ログは英語
- `capturer` 生成 IIFE の `if (args.no_video_device) return nullptr;` の順序は変えず、`RTCManager` 側の `if (video_track_source && !config_.no_video_device)` (`src/rtc/rtc_manager.cpp:288`) と併せて `no_video_device == true` ではビデオトラックが生成されない仕様を維持する。フェイクビデオを生成する案は `config_.no_video_device` の扱いと矛盾するため採用しない

## 完了条件

- `USE_FAKE_CAPTURE_DEVICE=ON` ビルド (`macOS` / `ubuntu-22.04_x86_64` / `ubuntu-24.04_x86_64`) で `--no-video-input-device --fake-capture-device sora --signaling-urls wss://example.com --channel-id test` (`no_audio_device` は未指定) を実行すると `exit 2` し `stderr` に英語メッセージが出力され、クラッシュしない
- `create_adm` 内の `!capturer` ガードにより、万一 CLI チェックを迂回しても `SetAudioCapturer` が呼ばれず `FakeAudioCapturer` 単体が返る
- `test/test_momo_validation.py` に異常系を追加し `subprocess` で `returncode` を assert する。`sendrecv` 正常系 E2E は回帰しない
- `CHANGES.md` の `develop` に `[FIX] --no-video-input-device と --fake-capture-device 併用時のクラッシュを修正` を追記する

## 解決方法

CLI で `--no-video-input-device` と `--fake-capture-device` の併用を禁止し、stderr に英語メッセージを出して終了コード 2 で返す。`--help` にも併用不可を追記した。`create_adm` では `capturer` が null なら `SetAudioCapturer` を呼ばない。`test_momo_validation.py` で終了コード 2 を検証する。CI のビルドと E2E は成功。PR は https://github.com/shiguredo/momo/pull/470 。
