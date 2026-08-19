# --no-video-input-device と --fake-capture-device の併用で nullptr メンバ呼び出しによりクラッシュする

- Created: 2026-08-19
- Completed: {YYYY-MM-DD}
- Branch: feature/fix-fake-capture-no-video-crash
- Polished: {YYYY-MM-DD}

## 目的

`--no-video-input-device` (ビデオキャプチャ無効) と `--fake-capture-device` (フェイクデバイス有効) を同時に指定した場合、`main.cpp` の `create_adm` ラムダ内で `capturer` が `nullptr` のまま `static_cast<FakeVideoCapturer*>(capturer.get())->SetAudioCapturer()` を呼び、確実にクラッシュする。これを修正する。

## 現状

- `src/main.cpp` の `args.fake_capture_device && !args.no_audio_device` 分岐内で `capturer` を null チェックせずに使用
- `capturer` は `args.no_video_device` が先にチェックされるため `nullptr` になりうる (`main.cpp` のキャプチャ生成 IIFE)
- `create_adm` は `src/rtc/rtc_manager.cpp` の worker スレッドで必ず呼ばれる

## 設計方針

- `capturer` が `nullptr` の場合、フェイクオーディオキャプチャをビデオキャプチャへ紐付けない
- `--no-video-input-device` と `--fake-capture-device` の組み合わせを CLI バリデーションで禁止するか、フェイクビデオを生成するようにする
- 方針は CLI の仕様として明示する

## 完了条件

- `--no-video-input-device` と `--fake-capture-device` の併用でクラッシュしない
- 併用時の挙動が仕様として明確で、エラーまたは正しい動作のどちらかに定まる
- 関連するテストが追加されている

## 解決方法

未着手 (PR 作成後に追記する)
