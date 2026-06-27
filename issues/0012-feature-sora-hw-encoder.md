# Sora モードの HW エンコーダー有効化 (NVIDIA NvCodec / Intel oneVPL / AMD AMF)

## 概要

sora_sdk は NVIDIA NvCodec (`nvcodec` feature)、Intel oneVPL (`vpl` feature)、
AMD AMF (`amf` feature) の VideoCodecCapability を内部実装し、それぞれ
`NvCodecVideoCodecCapability`、`VplVideoCodecCapability`、
`AmfVideoCodecCapability` として公開している。

しかし momo-rs 側で `sora_sdk` の `nvcodec` / `vpl` / `amf` feature を有効化しておらず、
Sora モードでこれらの HW エンコーダーを利用できない。

本 issue では以下を実施する:

1. `Cargo.toml` で `sora_sdk` の `nvcodec` / `vpl` / `amf` feature を有効化する
2. `src/sora/mod.rs` で NVIDIA / Intel / AMD 向けの codec preference 設定を追加する
3. `src/main.rs` の `print_video_codec_engines()` に NVIDIA / Intel / AMD を追加する

Apple VideoToolbox は既に実装済み (`apply_video_toolbox_preference()` +
`InternalAppleVideoCodecCapability`) のため対象外。

P2P / Ayame モードの HW エンコーダー対応は shiguredo_webrtc の VideoEncoderFactory
実装が必要で規模が大きいため別 issue とする。

## 現状

### sora_sdk 側 (2026.1.0-canary.11)

| 公開型 | feature | 依存クレート | 対応コーデック |
|---|---|---|---|
| `NvCodecVideoCodecCapability` | `nvcodec` | `shiguredo_nvcodec` | H.264, H.265, AV1 |
| `VplVideoCodecCapability` | `vpl` | `shiguredo_vpl` | VP9, AV1, H.264, H.265 |
| `AmfVideoCodecCapability` | `amf` | `shiguredo_amf` | H.264, H.265, AV1 |
| `InternalAppleVideoCodecCapability` | macOS/iOS 自動 | (ObjC default) | H.264, H.265 |

いずれも `VideoCodecCapability` trait を実装し、`SoraConnectionContextConfig` の
`video_codec_capabilities` に登録することで利用可能になる。

### momo-rs 側

- `Cargo.toml`: `sora_sdk = { version = "2026.1.0-canary.11", optional = true }`。
  feature 指定なしのため `default = ["openh264"]` のみ有効。`nvcodec` / `vpl` / `amf` は無効。
- `src/sora/mod.rs`:
  - `apply_video_toolbox_preference()` (L456) が `--h264-encoder videotoolbox` と
    `--h265-encoder videotoolbox` に対応。内部で `VideoCodecPreference` の
    `find_mut()` → `set_implementation()` を呼び出している。
  - NVIDIA / Intel / AMD 用の同等の preference 設定は存在しない。
- `src/main.rs`:
  - `--h264-encoder` / `--h265-encoder` / `--h264-decoder` / `--h265-decoder`
    (L303-318) はパース済みで `MomoConfig` に格納されている。
  - `--av1-encoder` / `--av1-decoder` (L293-300) は `_` プレフィックスで未使用
    (#0010 pending)。
  - `print_video_codec_engines()` (L897) が `InternalVideoCodecCapability` と
    `InternalAppleVideoCodecCapability` のみ列挙。NVIDIA / Intel / AMD 非表示。

## 必要な実装

### 1. Cargo.toml で sora_sdk の feature を有効化する

momo 側に `nvcodec` / `vpl` / `amf` feature を追加し、`sora_sdk` の同名 feature を転送する:

```toml
[features]
nvcodec = ["sora", "sora_sdk/nvcodec"]
vpl = ["sora", "sora_sdk/vpl"]
amf = ["sora", "sora_sdk/amf"]
```

各 feature とも `sora` を暗に有効化する（HW エンコーダーは Sora モードでのみ使用するため）。

各クレート (`shiguredo_nvcodec` / `shiguredo_vpl` / `shiguredo_amf`) は
ビルド時に GPU ドライバを要求しない（動的ロード方式）。

### 2. src/sora/mod.rs に NVIDIA / Intel / AMD 用 capability 登録と preference 設定を追加する

`SoraConnectionContextConfig::default()` は `InternalVideoCodecCapability` と
`InternalAppleVideoCodecCapability` のみを自動登録する。`nvcodec` / `vpl` / `amf` feature が
有効でも各 HW capability は自動追加されない。

そのため momo-rs 側で明示的に `ctx_config.video_codec_capabilities` に push する必要がある。
登録後、`apply_video_toolbox_preference()` と同様の preference 設定関数で
`--h264-encoder nvidia` 等の CLI 指定を反映する。

実装パターンは共通のため、`apply_video_toolbox_preference()` を汎用化する:

```rust
fn apply_codec_preference(
    preference: &mut VideoCodecPreference,
    direction: CodecDirection,
    codec_type: shiguredo_webrtc::VideoCodecType,
    value: Option<&str>,
    target: &str,
    implementation_name: &'static str,
    implementation_description: &'static str,
) -> Result<(), BoxError> {
    let Some(v) = value else { return Ok(()); };
    if v != target { return Ok(()); };
    let Some(codec) = preference.find_mut(direction, codec_type) else {
        return Err(format!(
            "video codec preference not found: direction={direction:?}, codec_type={codec_type:?}"
        ).into());
    };
    codec.set_implementation(VideoCodecImplementation::new(
        implementation_name,
        implementation_description,
    ));
    Ok(())
}
```

`sora::run()` 内、`SoraConnectionContextConfig` 構築箇所で以下のように
capability 登録と preference 設定を行う:

```rust
// NVIDIA NvCodec: capability 登録 + preference 設定
#[cfg(feature = "nvcodec")]
if let Ok(nvcodec) = NvCodecVideoCodecCapability::new() {
    // 内部の VideoCodecPreference に preference を事前登録する
    ctx_config.video_codec_preference.merge(
        &VideoCodecPreference::new_from_capability(&nvcodec),
    );
    ctx_config.video_codec_capabilities.push(Box::new(nvcodec));

    for (codec_type, encoder, decoder) in [
        (VideoCodecType::H264, config.h264_encoder.as_deref(), config.h264_decoder.as_deref()),
        (VideoCodecType::H265, config.h265_encoder.as_deref(), config.h265_decoder.as_deref()),
    ] {
        apply_codec_preference(&mut ctx_config.video_codec_preference, CodecDirection::Encoder, codec_type, encoder, "nvidia", "nvidia", "NVIDIA NVENC/NVDEC")?;
        apply_codec_preference(&mut ctx_config.video_codec_preference, CodecDirection::Decoder, codec_type, decoder, "nvidia", "nvidia", "NVIDIA NVENC/NVDEC")?;
    }
}

// Intel oneVPL: capability 登録 + preference 設定
#[cfg(feature = "vpl")]
if let Ok(vpl) = VplVideoCodecCapability::new() {
    ctx_config.video_codec_preference.merge(
        &VideoCodecPreference::new_from_capability(&vpl),
    );
    ctx_config.video_codec_capabilities.push(Box::new(vpl));

    for (codec_type, encoder, decoder) in [
        (VideoCodecType::VP9, config.vp9_encoder.as_deref(), config.vp9_decoder.as_deref()),
        (VideoCodecType::H264, config.h264_encoder.as_deref(), config.h264_decoder.as_deref()),
        (VideoCodecType::H265, config.h265_encoder.as_deref(), config.h265_decoder.as_deref()),
    ] {
        apply_codec_preference(&mut ctx_config.video_codec_preference, CodecDirection::Encoder, codec_type, encoder, "vpl", "vpl", "Intel oneVPL")?;
        apply_codec_preference(&mut ctx_config.video_codec_preference, CodecDirection::Decoder, codec_type, decoder, "vpl", "vpl", "Intel oneVPL")?;
    }
}

// AMD AMF: capability 登録 + preference 設定
#[cfg(feature = "amf")]
if let Ok(amf) = AmfVideoCodecCapability::new() {
    ctx_config.video_codec_preference.merge(
        &VideoCodecPreference::new_from_capability(&amf),
    );
    ctx_config.video_codec_capabilities.push(Box::new(amf));

    for (codec_type, encoder, decoder) in [
        (VideoCodecType::H264, config.h264_encoder.as_deref(), config.h264_decoder.as_deref()),
        (VideoCodecType::H265, config.h265_encoder.as_deref(), config.h265_decoder.as_deref()),
    ] {
        apply_codec_preference(&mut ctx_config.video_codec_preference, CodecDirection::Encoder, codec_type, encoder, "amf", "amf", "AMD AMF")?;
        apply_codec_preference(&mut ctx_config.video_codec_preference, CodecDirection::Decoder, codec_type, decoder, "amf", "amf", "AMD AMF")?;
    }
}
```

`NvCodecVideoCodecCapability::new()` / `VplVideoCodecCapability::new()` /
`AmfVideoCodecCapability::new()` は GPU 非搭載環境でエラーを返す。
その場合は capability 登録をスキップする（ログ出力不要）。
`new_from_capability()` で生成した preference を `merge()` することで、
`apply_codec_preference()` が `find_mut()` で該当 codec を見つけられるようになる。

VP9/AV1 の CLI オプション (`--vp9-encoder` / `--vp9-decoder` / `--av1-encoder` / `--av1-decoder`)
は現在 `_` プレフィックスで未使用のため、本 issue では Sora モードに必要なものに限り
`_` を除去し `SoraConfig` にもフィールドを追加する。
HW バックエンド未整備を理由とする `_` プレフィックス解除の一般論は #0010 に委ねる。

現時点で Sora モードの HW エンコーダー向けに必要で `_` を外すべきオプション:
- `--vp9-encoder` / `--vp9-decoder` (Intel VPL の VP9 対応に必要)
- `--av1-encoder` / `--av1-decoder` (NVIDIA/Intel/AMD の AV1 対応に必要)

### 3. print_video_codec_engines() に NVIDIA / Intel / AMD を追加する

`src/main.rs:897` の `print_video_codec_engines()` に以下を追加する:

```rust
// NVIDIA NvCodec
#[cfg(feature = "nvcodec")]
{
    use sora_sdk::NvCodecVideoCodecCapability;
    if let Ok(cap) = NvCodecVideoCodecCapability::new() {
        capabilities.push(Box::new(cap));
    }
}

// Intel oneVPL
#[cfg(feature = "vpl")]
{
    use sora_sdk::VplVideoCodecCapability;
    if let Ok(cap) = VplVideoCodecCapability::new() {
        capabilities.push(Box::new(cap));
    }
}

// AMD AMF
#[cfg(feature = "amf")]
{
    use sora_sdk::AmfVideoCodecCapability;
    if let Ok(cap) = AmfVideoCodecCapability::new() {
        capabilities.push(Box::new(cap));
    }
}
```

各 `new()` は `sora_sdk::Result<Self>` を返す。GPU 非搭載環境ではエラーになるため
`if let Ok(cap)` で分岐する（エラー時は単にスキップ、ログ出力も不要）。

### 4. E2E テスト

以下の E2E テストが既に存在し、環境変数でスキップ制御されている:

- `e2e-tests/test_sora_mode_nvidia_video_codec.py` (`NVIDIA_VIDEO_CODEC` 環境変数)
- `e2e-tests/test_sora_mode_intel_vpl.py` (`INTEL_VPL` 環境変数)

本実装によりこれらのテストが動作可能になる。CI でのテスト実行は
対応ハードウェアを持つ self-hosted runner が必要なため別途検討する。

## 対象外（別 issue）

- **P2P / Ayame モードの HW エンコーダー対応**:
  shiguredo_webrtc の VideoEncoderFactory trait 実装が必要で
  作業規模が大きく技術スタックも異なるため別 issue に分割する
- **VP8/VP9/AV1 encoder/decoder オプションの一般有効化**: #0010 (pending) で対応


## 参考

- sora_sdk `src/video_codecs/nvcodec.rs`: `NvCodecVideoCodecCapability`
- sora_sdk `src/video_codecs/vpl.rs`: `VplVideoCodecCapability`
- sora_sdk `src/lib.rs:46`: `pub use crate::video_codecs::amf::AmfVideoCodecCapability`
- sora_sdk `src/lib.rs:55`: `pub use crate::video_codecs::nvcodec::NvCodecVideoCodecCapability`
- sora_sdk `src/lib.rs:61`: `pub use crate::video_codecs::vpl::VplVideoCodecCapability`
- sora_sdk `src/video_codec_preference.rs`: `VideoCodecPreference`, `PreferenceCodec`
- sora_sdk `Cargo.toml` features: `nvcodec = ["dep:shiguredo_nvcodec"]`, `vpl = ["dep:shiguredo_vpl"]`, `amf = ["dep:shiguredo_amf"]`
- sora_sdk `src/connection_context.rs:28-63`: `SoraConnectionContextConfig::default()` の capability 自動登録
- 既存実装: `src/sora/mod.rs:456-477` `apply_video_toolbox_preference()`
- 既存実装: `src/main.rs:897-932` `print_video_codec_engines()`
- 既存 E2E テスト: `e2e-tests/test_sora_mode_nvidia_video_codec.py`, `e2e-tests/test_sora_mode_intel_vpl.py`
  - AMD AMF の E2E テストは未整備のため、本実装後に別途追加を検討する
- Polished: 2026-06-28
