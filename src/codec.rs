//! コーデックプリファレンスの共通処理

use shiguredo_webrtc::{MediaType, RtpCodecCapabilityVector};
use tracing::info;

use crate::error::BoxError;

/// 補助コーデックかどうかを判定する
fn is_auxiliary_codec(name: &str, media_type: &MediaType) -> bool {
    let name_lower = name.to_lowercase();
    match media_type {
        MediaType::Video => matches!(name_lower.as_str(), "rtx" | "red" | "ulpfec" | "flexfec-03"),
        MediaType::Audio => matches!(name_lower.as_str(), "telephone-event" | "cn" | "red"),
        _ => false,
    }
}

/// Transceiver にコーデックプリファレンスを設定する
///
/// PeerConnectionFactory から送受信両方の capabilities を取得し、
/// 指定コーデックを primary、補助コーデックを secondary としてフィルタリングする。
pub(crate) fn set_codec_preferences(
    factory: &shiguredo_webrtc::PeerConnectionFactory,
    audio_transceiver: &mut shiguredo_webrtc::RtpTransceiver,
    video_transceiver: Option<&mut shiguredo_webrtc::RtpTransceiver>,
    video_codec_type: Option<&str>,
    audio_codec_type: Option<&str>,
) -> Result<(), BoxError> {
    // 映像コーデックプリファレンス
    if let (Some(target_codec), Some(transceiver)) = (video_codec_type, video_transceiver) {
        let filtered = filter_codecs(factory, MediaType::Video, target_codec)?;
        transceiver
            .set_codec_preferences(&filtered)
            .map_err(crate::error::wrtc_err)?;
        info!(target: "codec", codec = target_codec, "video codec preferences set");
    }

    // 音声コーデックプリファレンス
    if let Some(target_codec) = audio_codec_type {
        let filtered = filter_codecs(factory, MediaType::Audio, target_codec)?;
        audio_transceiver
            .set_codec_preferences(&filtered)
            .map_err(crate::error::wrtc_err)?;
        info!(target: "codec", codec = target_codec, "audio codec preferences set");
    }

    Ok(())
}

/// 送受信の capabilities から共通コーデックを抽出し、指定コーデックでフィルタリングする
fn filter_codecs(
    factory: &shiguredo_webrtc::PeerConnectionFactory,
    media_type: MediaType,
    target_codec: &str,
) -> Result<RtpCodecCapabilityVector, BoxError> {
    let sender_caps = factory.get_rtp_sender_capabilities(media_type.clone());
    let receiver_caps = factory.get_rtp_receiver_capabilities(media_type.clone());
    let sender_codecs = sender_caps.codecs();
    let receiver_codecs = receiver_caps.codecs();

    // 送受信両方でサポートされている共通コーデックを求める
    let mut common_indices = Vec::new();
    for i in 0..sender_codecs.len() {
        let Some(s_cap) = sender_codecs.get(i) else {
            continue;
        };
        let Ok(s_name) = s_cap.name() else { continue };
        for j in 0..receiver_codecs.len() {
            let Some(r_cap) = receiver_codecs.get(j) else {
                continue;
            };
            let Ok(r_name) = r_cap.name() else { continue };
            if s_name.eq_ignore_ascii_case(&r_name) {
                common_indices.push(i);
                break;
            }
        }
    }

    if common_indices.is_empty() {
        return Err("no common codec capabilities available".into());
    }

    // primary (指定コーデック) と auxiliary (補助コーデック) に分類
    let mut filtered = RtpCodecCapabilityVector::new(0);
    // まず primary コーデックを追加
    let mut found_primary = false;
    for &idx in &common_indices {
        let Some(cap) = sender_codecs.get(idx) else {
            continue;
        };
        let Ok(name) = cap.name() else { continue };
        if name.eq_ignore_ascii_case(target_codec) {
            filtered.push(&cap);
            found_primary = true;
        }
    }
    if !found_primary {
        return Err(format!(
            "specified codec '{target_codec}' is not available for {media_type:?}"
        )
        .into());
    }
    // 次に auxiliary コーデックを追加
    for &idx in &common_indices {
        let Some(cap) = sender_codecs.get(idx) else {
            continue;
        };
        let Ok(name) = cap.name() else { continue };
        if is_auxiliary_codec(&name, &media_type) {
            filtered.push(&cap);
        }
    }

    info!(
        target: "codec",
        codec = target_codec,
        count = filtered.len(),
        "filtered codec capabilities"
    );

    Ok(filtered)
}
