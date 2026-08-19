# README の libwebrtc バージョン表記と著作権年を実態に合わせる

- Created: 2026-08-19
- Completed: {YYYY-MM-DD}
- Branch: feature/fix-readme-version
- Polished: {YYYY-MM-DD}
- Milestone: 2026.1.0

## 目的

`README.md` の libwebrtc バッジが m138 のまま残っており、実際の m150 と乖離している。リンク先 (`branch-heads/7204`) も誤誘導になる。また著作権年が 2025 のまま。正式リリース前に実態へ合わせる。

## 現状

- `README.md` (3 行): `[![libwebrtc](https://img.shields.io/badge/libwebrtc-m138.7204-blue.svg)](...)` が m138 のまま
- `CHANGES.md` の develop セクションでは m150.7871.3.0 に更新済み
- `README.md` (172-173 行): `Copyright 2015-2025` のまま

## 設計方針

- libwebrtc バッジを m150.7871.3.0 に更新し、リンク先も対応する branch-heads へ修正する
- 著作権年を 2026 に更新する

## 完了条件

- README の libwebrtc 表記が実態 (m150) と一致する
- 著作権年が 2026 になる

## 解決方法

未着手 (PR 作成後に追記する)
