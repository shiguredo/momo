# libwebrtc のバージョンを m150 に上げる

- Priority: Medium
- Created: 2026-06-03
- Model: Opus 4.8
- Branch: feature/update-libwebrtc-m150

## 目的

libwebrtc のバージョンを現在の `m138.7204.0.4` から m150 系へ更新する。

libwebrtc を新しいバージョンへ追従することで、上流の WebRTC のバグ修正・セキュリティ修正・機能追加を取り込み、Chromium 本体との互換性を保つ。バージョンが古いまま放置すると、ビルドツールチェインや依存ライブラリとの乖離が広がり、将来の更新コストが増大する。

## 優先度根拠

- Medium とする。
- libwebrtc の追従は momo の根幹となる依存であり技術的重要性は高いが、m138 でも現状の機能は動作しているため緊急性は High ではない。
- 一方で m138 から m150 は milestone 12 個分の差があり、放置するほど API 変更の追従コストが膨らむため、優先的に対応すべき Medium とする。

## 現状

- libwebrtc のバージョンは `DEPS` の `WEBRTC_BUILD_VERSION` で管理されている。

```
WEBRTC_BUILD_VERSION=m138.7204.0.4
```

- ビルド時は `run.py` が `DEPS` の `WEBRTC_BUILD_VERSION` を読み取り、`shiguredo-webrtc-build/webrtc-build` の対応するリリースを取得する（`buildbase.py` の `install_webrtc()`）。
  - URL: `https://github.com/shiguredo-webrtc-build/webrtc-build/releases/download/{version}/{filename}`
- `run.py` は取得した `webrtc.version` から `WEBRTC_BUILD_VERSION` / `WEBRTC_READABLE_VERSION` / `WEBRTC_COMMIT` を CMake 引数として渡す（`run.py:516-518`）。

## 前提条件

- 2026-06-03 時点で `shiguredo-webrtc-build/webrtc-build` の最新リリースは m149 系（m149.7827.4.0）であり、**m150 系のリリースはまだ存在しない**。
- そのため本 issue は m150 系のリリースが `shiguredo-webrtc-build/webrtc-build` で公開されてから着手する。
- 着手時に公開されている m150 系の最新パッチバージョンを採用し、本 issue の完了条件に具体的なバージョン文字列を確定して記載する。

## 設計方針

- `DEPS` の `WEBRTC_BUILD_VERSION` を m150 系の最新リリースへ更新する。
- libwebrtc の API 変更に追従する。m138 から m150 への更新では、上流の WebRTC API のシグネチャ変更・削除・名前変更が発生している可能性が高いため、ビルドエラーを 1 つずつ解消する。
  - 影響を受ける可能性が高いのは `src/rtc/` 配下の WebRTC API を直接利用しているコード。
- ビルドツールチェイン（Clang / libc++ など）の追従が必要な場合は `webrtc-build` の `webrtc.version` に従う。
- 各プラットフォーム（macOS / Ubuntu / Windows / Raspberry Pi 等）でビルドが通ることを確認する。

## 完了条件

- `DEPS` の `WEBRTC_BUILD_VERSION` が m150 系の最新リリースに更新されている。
  - 具体的なバージョン文字列は着手時に確定する（例: `m150.xxxx.x.x`）。
- 全対応プラットフォームで momo がビルドできる。
- libwebrtc の API 変更への追従が完了し、ビルド警告・エラーが解消されている。
- 既存の動作（映像・音声の送受信、各エンコーダー等）に回帰がないことを確認する。
- `CHANGES.md` の `## develop` セクションに `[UPDATE] libwebrtc のバージョンを m150 系に上げる` を追記する。

## 解決方法

1. `shiguredo-webrtc-build/webrtc-build` で m150 系の最新リリースを確認する。
2. `DEPS` の `WEBRTC_BUILD_VERSION` を当該バージョンへ更新する。
3. ビルドを実行し、libwebrtc の API 変更によるエラーを `src/` 配下のコードで解消する。
4. 各プラットフォームでビルド・動作確認を行う。
5. `CHANGES.md` を更新する。
