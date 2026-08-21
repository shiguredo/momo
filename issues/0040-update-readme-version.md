# README の libwebrtc バージョン表記と著作権年を実態に合わせる

- Created: 2026-08-19
- Completed: {YYYY-MM-DD}
- Branch: feature/update-readme-version
- Polished: 2026-08-21
- Milestone: 2026.1.0

## 目的

`README.md` の libwebrtc バッジが m138 のまま残っており、実際の m150 と乖離している。リンク先 (`branch-heads/7204`) も m138 を指すため誤誘導になる。また著作権年が 2025 のまま。正式リリース前に実態へ合わせる。

## 現状

- `README.md` 先頭の libwebrtc バッジ: `[![libwebrtc](https://img.shields.io/badge/libwebrtc-m138.7204-blue.svg)](https://chromium.googlesource.com/external/webrtc/+/branch-heads/7204)` が m138 のまま
- `CHANGES.md` の develop セクションでは m150.7871.3.0 に更新済み (`DEPS` の `WEBRTC_BUILD_VERSION=m150.7871.3.0`)
- `README.md` ライセンスセクションの Copyright ブロック: `Copyright 2015-2025, tnoho (Original Author)` / `Copyright 2018-2025, Shiguredo Inc.` が 2025 のまま

## 設計方針

- libwebrtc バッジを `libwebrtc-m150.7871.3.0` に更新し、リンク先を `https://chromium.googlesource.com/external/webrtc/+/branch-heads/7871` に修正する。M150 は WebRTC の branch-heads/7871 に対応する (現行の m138 = 7204 も同じ対応関係)
- 著作権年を 2026 に更新する (現在年 2026)

## 完了条件

- README の libwebrtc バッジの本文 (`m150.7871.3.0`) とリンク先 (`branch-heads/7871`) が実態と一致する
- 著作権年が 2026 になる

## 解決方法

未着手 (PR 作成後に追記する)
