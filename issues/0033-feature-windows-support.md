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

## 依存クレートの Windows 対応状況 (2026-05-04 調査)

- `shiguredo_webrtc`: Windows 11 / Windows Server 2025 x86_64 をサポート (VS2022 + Clang tools 必須)
- `shiguredo_audio_device`: Windows x86_64 をサポート (WASAPI、追加依存なし)
- `shiguredo_video_device`: Windows x86_64 をサポート (Media Foundation、追加依存なし)
- `shiguredo_openh264`: Windows 対応 (`LoadLibraryW` で動的ロード)
- `raden`: x86_64-pc-windows-msvc をサポート
- `raw_player`: Windows 11 / Windows Server 2025 x86_64 をサポート (デフォルトはプリビルド SDL3)
- `sora_sdk`: Windows 11 / Windows Server 2025 x86_64 をサポート

## 必要な実装

- [x] GitHub Actions に Windows ビルドジョブを追加 (windows-2025, ayame + sora feature)
- [ ] Windows 実機での動作確認 (カメラ / マイク / スピーカー / 映像表示)
- [ ] シリアル DataChannel の Windows 対応 (serialport-rs への移行を含む別 issue 化を検討)
- [ ] `metrics::get_os_detail()` の Windows 詳細取得実装 (現在は `std::env::consts::OS` を返すだけ)
- [ ] リリースアーティファクトに Windows バイナリを含める (リリースワークフロー自体が未整備)

## 進捗

- 2026-05-04: 依存クレートの Windows 対応状況を確認し、`.github/workflows/ci.yml` に `windows-build` ジョブを追加。`cargo clippy / build / test` を windows-2025 上で実行する。

## 根拠

momo (C++) と同等のプラットフォーム対応を目指すため。Windows は商用導入の主要プラットフォームの一つ。

## 参考

- momo-cpp の Windows ビルド設定 (`run.py` の windows_x86_64)
- shiguredo_webrtc の Windows サポート: <https://docs.rs/crate/shiguredo_webrtc/latest>
- shiguredo_audio_device の Windows サポート: <https://docs.rs/crate/shiguredo_audio_device/latest>
- shiguredo_video_device の Windows サポート: <https://docs.rs/crate/shiguredo_video_device/latest>
