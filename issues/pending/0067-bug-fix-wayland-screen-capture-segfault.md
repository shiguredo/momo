# Ubuntu の Wayland 環境で --screen-capture を使用すると momo がセグフォする

- Created: 2026-09-11
- Completed: {YYYY-MM-DD}
- Branch: feature/fix-wayland-screen-capture-segfault
- Polished: {YYYY-MM-DD}
- Reporter: @torikizi

## 目的

Ubuntu の Wayland 環境で `--screen-capture` を使用すると momo がセグメンテーション違反で落ちる。原因は上流の libwebrtc の不具合で、上流では修正済みのため、libwebrtc を更新したバイナリで解消するかを検証する。

## 現状

- momo 2024.1.2 で `--screen-capture` を使用すると Wayland 環境でセグメンテーション違反が発生することを確認した
- 原因は libwebrtc の "Linux Wayland Screenshare Crash" (<https://issues.webrtc.org/issues/420959042>) で、上流では修正済み
- momo は libwebrtc の更新で修正を取り込む。現行の develop は libwebrtc m150.7871.3.0 を使用している (`README.md` / `CHANGES.md`)
- `--screen-capture` は CMake の `USE_SCREEN_CAPTURER` が ON のときに有効になり、`src/main.cpp` から `ScreenVideoCapturer` を生成して `webrtc::DesktopCapturer::CreateScreenCapturer` を呼ぶ
- Wayland でのスクリーンキャプチャを継続的に検証する CI 環境はなく、手動確認が必要

## 検証方針

- libwebrtc を更新した現行の develop バイナリを Wayland 環境で実行し、`--screen-capture` でセグメンテーション違反が発生しないことを確認する
- X11 環境でも従来どおり動作することを確認する
- 解消しない場合は上流の修正が momo の利用する libwebrtc に含まれているかを確認し、追加で追従する
- 検証結果を本 issue に記録する

## 完了条件

- Wayland 環境で `--screen-capture` を指定して momo が起動し、スクリーンキャプチャの映像が送信できる
- X11 環境で `--screen-capture` が従来どおり動作する
- 検証した libwebrtc のバージョンと環境が記録されている

## 解決方法

未着手 (検証完了後に追記する)

## pending にした理由

原因は上流の libwebrtc の不具合で、上流では修正済みのため momo 側のコード変更では対応しない。libwebrtc を更新したバイナリを Wayland 環境で検証する必要があるが、Wayland 環境での検証は CI になく手動確認となり、次回リリース時に確認するため pending とする。
