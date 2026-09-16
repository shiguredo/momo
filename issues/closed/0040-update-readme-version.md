# README の libwebrtc バージョン表記と著作権年を実態に合わせる

- Created: 2026-08-19
- Completed: 2026-08-21
- Branch: feature/update-readme-version
- Polished: 2026-08-21
- Milestone: 2026.1.0

## 目的

`README.md` の libwebrtc バッジが m138 のまま残っており、実際の m150 と乖離している。リンク先 (`branch-heads/7204`) も m138 を指すため誤誘導になる。また著作権表記が年範囲 (`2015-2025` / `2018-2025`) のままである。正式リリース前に実態へ合わせる。

## 現状

- `README.md` 先頭の libwebrtc バッジ: `[![libwebrtc](https://img.shields.io/badge/libwebrtc-m138.7204-blue.svg)](https://chromium.googlesource.com/external/webrtc/+/branch-heads/7204)` が m138 のまま
- `CHANGES.md` の develop セクションでは m150.7871.3.0 に更新済み (`DEPS` の `WEBRTC_BUILD_VERSION=m150.7871.3.0`)
- `README.md` ライセンスセクションの Copyright ブロック: `Copyright 2015-2025, tnoho (Original Author)` / `Copyright 2018-2025, Shiguredo Inc.` が年範囲表記のまま

## 設計方針

- libwebrtc バッジを `libwebrtc-m150.7871.3.0` に更新し、リンク先を `https://chromium.googlesource.com/external/webrtc/+/branch-heads/7871` に修正する。M150 は WebRTC の branch-heads/7871 に対応する (現行の m138 = 7204 も同じ対応関係)
- 著作権表記は年範囲をやめ、初出年のみにする。形式は次のとおりとする (年と権利者のあいだにカンマは付けない)

```text
Copyright 2015 tnoho (Original Author)
Copyright 2018 Shiguredo Inc.
```

## 完了条件

- README の libwebrtc バッジの本文 (`m150.7871.3.0`) とリンク先 (`branch-heads/7871`) が実態と一致する
- 著作権表記が初出年のみ (`Copyright 2015 tnoho (Original Author)` / `Copyright 2018 Shiguredo Inc.`) になっている

## 解決方法

- `README.md` 先頭の libwebrtc バッジを `m150.7871.3.0` に更新し、リンク先を `branch-heads/7871` に変更した
- ライセンスセクションの著作権表記を年範囲から初出年のみに変更した

```text
Copyright 2015 tnoho (Original Author)
Copyright 2018 Shiguredo Inc.
```

- PR: https://github.com/shiguredo/momo/pull/461
