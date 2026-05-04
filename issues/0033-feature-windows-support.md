## Windows プラットフォーム対応

Created: 2026-05-04
Model: Opus 4.7

## 概要

momo-rs の Windows (x86_64) ビルドおよび動作対応。momo (C++) は windows_x86_64 をサポートしているが、momo-rs は対応していない。

## 現状

- `Cargo.toml` の各 feature が Linux / macOS 想定で記述されている
- シリアル DataChannel が `#[cfg(target_os = "linux")]` で gate されている (Windows では使えない)
- libcamera 関連は Linux 限定で問題なし
- Windows 向けの GitHub Actions ワークフローが存在しない

## momo の対応状況

- ビルドターゲット: `windows_x86_64` (`run.py AVAILABLE_TARGETS`)
- VS2022 + WebRTC ビルド済みバイナリでビルド
- DirectShow / Media Foundation 経由のカメラ取得
- WASAPI 経由の音声入出力

## 必要な実装

- shiguredo_webrtc の Windows ビルド対応の確認
- Windows 向けカメラキャプチャ (Media Foundation 等)
- Windows 向け音声入出力 (cpal の WASAPI バックエンド)
- シリアル DataChannel の Windows 対応 (serialport-rs は Windows 対応済み)
- GitHub Actions に Windows ビルドジョブを追加
- リリースアーティファクトに Windows バイナリを含める

## 根拠

momo (C++) と同等のプラットフォーム対応を目指すため。Windows は商用導入の主要プラットフォームの一つ。

## 参考

- momo-cpp の Windows ビルド設定 (`run.py` の windows_x86_64)
- shiguredo_webrtc の Windows サポート状況 (要確認)
