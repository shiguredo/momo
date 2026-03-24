# --fixed-resolution / --priority の実装

Created: 2026-03-22
Model: Opus 4.6

## 概要

映像品質劣化時の制御オプションがパースされるが未使用。

## 現状

- `src/main.rs` で `_fixed_resolution`, `_priority` に束縛されるだけで未使用
- CommonConfig に含まれない

## 根拠

momo は映像品質劣化時の挙動を `--fixed-resolution` と `--priority` で制御できる。
momo-rs ではこれらのオプションが CLI でパースされるだけで WebRTC に反映されていない。

## 必要な実装

- `--fixed-resolution`: 映像劣化時に解像度を維持する (`DegradationPreference::MaintainResolution` に相当)
- `--priority`: 以下の選択肢を `DegradationPreference` にマッピングする
  - `BALANCE` → `DegradationPreference::Balanced`
  - `FRAMERATE` → `DegradationPreference::MaintainFramerate`
  - `RESOLUTION` → `DegradationPreference::MaintainResolution`

### W3C RTCDegradationPreference と shiguredo_webrtc の対応

| W3C spec | shiguredo_webrtc | 意味 |
|----------|-----------------|------|
| `maintain-framerate` | `MaintainFramerate` | 解像度を下げてフレームレートを維持する |
| `maintain-resolution` | `MaintainResolution` | フレームレートを下げて解像度を維持する |
| `balanced` | `Balanced` | フレームレートと解像度をバランスよく劣化させる |
| `maintain-framerate-and-resolution` | `Disabled` | フレームレートも解像度も劣化させない。ネットワーク/エンコーダリソース超過時はエンコード前にフレームをドロップする |

設定方法:

1. `RtpSender::get_parameters()` で `RtpParameters` を取得
2. `RtpParameters::set_degradation_preference(Some(pref))` で設定
3. `RtpSender::set_parameters(&params)` で適用

### momo との対応

- `--fixed-resolution` 指定時: `MaintainResolution` を強制 (`--priority` より優先)
- `--priority BALANCE`: `Balanced`
- `--priority FRAMERATE`: `MaintainFramerate` (momo のデフォルト)
- `--priority RESOLUTION`: `MaintainResolution`

## 解決方法

- `_fixed_resolution` / `_priority` のアンダースコアを外して有効化
- `--priority` のバリデーション (BALANCE / FRAMERATE / RESOLUTION)
- `--fixed-resolution` は `--priority` より優先して `MaintainResolution` を強制
- P2P モード: `WebRtcEngine` に `degradation_preference` を保持し、`add_track` 後に `RtpSender::set_parameters()` で設定
- Ayame モード: `add_transceivers` と `handle_offer` で `add_track` 後に同様に設定
- Sora モード: sora_sdk に DegradationPreference API がないため未対応 (sora_sdk 側の API 追加が必要)

## 旧 pending 理由 (解消済み)

shiguredo_webrtc の DegradationPreference API が利用可能かの確認が必要としていたが、
shiguredo_webrtc v0.146.0-canary.4 で `DegradationPreference` 列挙型と
`RtpParameters::set_degradation_preference()` API が利用可能であることを確認した。

Completed: 2026-03-22 (P2P / Ayame モード)
