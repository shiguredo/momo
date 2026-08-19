# 英語コメントを日本語に統一する (AGENTS.md 規約違反の解消)

- Created: 2026-08-19
- Completed: {YYYY-MM-DD}
- Branch: feature/refactor-japanese-comments
- Polished: {YYYY-MM-DD}

## 目的

AGENTS.md の「コメントは全て日本語にすること」に違反する英語コメントが、boost::beast の example や WebRTC 由来のコピペコードを中心に多数残っている。正式リリース前に規約に沿って日本語へ統一する。

## 現状

- boost::beast サンプル由来の英語コメント
  - `src/sora/sora_server.cpp` (18, 25, 32, 39 行)
  - `src/sora/sora_session.cpp` (24-25, 28, 39, 119, 127, 131 行)
  - `src/p2p/p2p_server.cpp` (26, 33, 40, 47 行)
  - `src/p2p/p2p_session.cpp` (37, 43-44, 47, 88, 94, 99, 110, 115, 119, 123 行)
  - `src/metrics/metrics_server.cpp` (16, 23, 30, 37 行)
  - `src/metrics/metrics_session.cpp` (31, 37-38, 41 行)
- WebRTC / sora-cpp-sdk 由来の英語コメント
  - `src/sora-cpp-sdk/src/hwenc_jetson/jetson_v4l2_capturer.cpp` 多数
  - `src/sora-cpp-sdk/src/open_h264_video_encoder.cpp` (コメント 126 個の大半)
  - `src/sora-cpp-sdk/src/hwenc_vpl/vpl_video_encoder.cpp` 他
  - `src/rtc/momo_video_encoder_factory.cpp` (298 行)
- 注意: `src/sora-cpp-sdk/` は vendored コードのため、同期元 (sora-cpp-sdk 2026.2.1) で既に日本語化済みの場合は同期で解消される。同期後も残る英語コメントのみ momo 側で修正する

## 設計方針

- まず `0013-update-sora-cpp-sdk-sdl3-cli11` の同期で解消されるかを確認する
- 同期後も残る英語コメントを日本語へ書き換える
- ログメッセージ (英語必須) と混同しないこと

## 完了条件

- momo 固有のソース (`src/` 直下、`src/sora-cpp-sdk/` の momo 側変更分) に英語コメントが無い
- コメントは全て日本語である

## 解決方法

未着手 (PR 作成後に追記する)
