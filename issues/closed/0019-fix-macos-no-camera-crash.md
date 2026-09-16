# カメラが 0 台の macOS で起動時に NSRangeException が投げられクラッシュする

- Created: 2026-08-19
- Completed: 2026-08-27
- Branch: feature/fix-macos-no-camera-crash
- Polished: 2026-08-19
- Milestone: 2026.1.0

## 目的

macOS でカメラが 1 台も存在しない環境 (VM・ヘッドレス・プライバシー設定で拒否、macOS 14 では External を含めて 0 台) で Momo を起動すると、`src/mac_helper/mac_capturer.mm` の `MacCapturer::FindVideoDevice` が空の `devices` 配列に対して `[devices objectAtIndex:capture_device_index]` を呼び `NSRangeException` (Objective-C 例外) が投げられる。これは C++ 側の `MacCapturer::Create` 経由で同期的に呼ばれ `@try/@catch` が無いため `std::terminate` でプロセスが落ちる。これを修正する。

## 現状

- `src/mac_helper/mac_capturer.mm` の `MacCapturer::FindVideoDevice` で、ビデオデバイス指定が無い / `default` / `0` の場合に `capture_device_index = 0` とし、`devices.count == 0` のチェックなしに `[devices objectAtIndex:capture_device_index]` を呼ぶ。`Create` は `FindVideoDevice` を呼ぶだけで直接 `objectAtIndex:` を呼ばない
- `devices` は `captureDevices()` (`src/mac_helper/mac_capturer.mm:140` の `session.devices`) から取得され、カメラ 0 台の環境では空 `NSArray` (`nil` ではなく `count == 0`) になる。`@available(macOS 14, *)` で External を含める分岐があっても 0 台は起こり得る
- `--list-devices` は `MacCapturer::GetVideoDeviceInfos()` (`src/mac_helper/mac_capturer.mm:121`) が `captureDevices()` を列挙するだけで `objectAtIndex:` を使わず、空配列でも `src/main.cpp:123` でヘッダのみ表示してクラッシュしない。`ListDevices()` は空を想定した分岐を持つ
- `src/main.cpp:277` には既に `if (!capturer && !args.no_video_device) { std::cerr << "failed to create capturer" << std::endl; return 1; }` の失敗フローがあり、`FindVideoDevice` が `nullptr` を返せば `return 1` で終了する。`--no-video-input-device` 指定時は `src/main.cpp:193` で `FindVideoDevice` を呼ばずに `nullptr` を返して回避可能

## 設計方針

- `MacCapturer::FindVideoDevice` 内の `auto devices = captureDevices();` 直後に `if (devices.count == 0) { RTC_LOG(LS_ERROR) << "Failed to create MacCapture: no video device found. Use --list-devices to check available devices or --no-video-input-device to run without camera"; return nullptr; }` を追加する。ログは英語、コメントは日本語。`Create` が `nullptr` を返し `src/main.cpp:277` の既存 `failed to create capturer` で `return 1` に至る流れとし、新たに `exit` は呼ばない
- 堅牢性のため最終分岐を `if (capture_device_index != SIZE_T_MAX && capture_device_index < devices.count)` に強化し、将来の範囲外指定でも `NSRangeException` を防ぐ
- 例外の捕捉 (`@try` / `@catch`) ではなく、事前に `devices.count` と `capture_device_index` の範囲を検証する方法で対応する。Objective-C 例外は C++ 側で安全に捕捉できないため
- `--list-devices` は `GetVideoDeviceInfos()` が空配列を正常処理するため修正せず維持する。`--no-video-input-device` での回避はエラーメッセージに含めて案内する

## 完了条件

- カメラ 0 台の macOS (VM または `Privacy & Security > Camera` 拒否) で起動しても `NSRangeException` でクラッシュせず、`RTC_LOG(LS_ERROR)` に英語で `no video device found` を含むメッセージ、`stderr` に `failed to create capturer` が出力され `exit 1` で終了する
- `--list-devices` はカメラ 0 台でも空一覧を正常に表示し (`=== Available video devices ===` ヘッダのみ)、クラッシュしない
- カメラ 1 台以上の環境では従来通り `selected video device: [0]` がログに出て起動する
- `CHANGES.md` の `develop` に `- [FIX] macOS でカメラ 0 台のときにクラッシュする問題を修正` を追記し、担当者を併記する

## 解決方法

`MacCapturer::FindVideoDevice` で `captureDevices()` の直後に `devices.count == 0` なら英語ログを出して `nullptr` を返す。`objectAtIndex:` は `capture_device_index < devices.count` のときだけ呼ぶ。`Create` が `nullptr` を返せば既存の `failed to create capturer` で終了コード 1 になる。カメラ権限拒否では一覧が空にならないことを確認した。0 台の再現はカメラ無し VM 等が必要。CI のビルドと E2E は成功。PR は https://github.com/shiguredo/momo/pull/469 。
