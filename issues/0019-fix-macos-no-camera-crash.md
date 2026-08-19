# カメラが 0 台の macOS で起動時に NSRangeException が投げられクラッシュする

- Created: 2026-08-19
- Completed: {YYYY-MM-DD}
- Branch: feature/fix-macos-no-camera-crash
- Polished: {YYYY-MM-DD}

## 目的

macOS でカメラが 1 台も存在しない環境 (VM・ヘッドレス・プライバシー設定で拒否) で Momo を起動すると、`mac_capturer.mm` が空の `devices` 配列に対して `objectAtIndex:0` を呼び `NSRangeException` (Objective-C 例外) が投げられる。これは C++ 側で捕捉されず `std::terminate` でプロセスが落ちる。これを修正する。

## 現状

- `src/mac_helper/mac_capturer.mm` の `MacCapturer::Create` で、指定が無い / `default` / `0` の場合に `capture_device_index = 0` とし、`devices.count == 0` のチェックなしに `[devices objectAtIndex:capture_device_index]` を呼ぶ

## 設計方針

- `devices.count == 0` の場合、エラーメッセージを出力して起動を失敗させる (exit または `--list-devices` 相当の案内)
- `--list-devices` は空でも正常に空一覧を表示する (既存動作を維持)
- 例外の捕捉ではなく、事前に配列の要素数を確認する方法で対応する

## 完了条件

- カメラ 0 台の macOS でクラッシュしない
- エラーメッセージが出力され、ユーザーが原因を理解できる
- カメラがある環境では従来通り動作する

## 解決方法

未着手 (PR 作成後に追記する)
