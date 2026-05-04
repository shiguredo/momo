//! Sora モードの実装
//!
//! sora_sdk を使用して WebRTC SFU Sora に接続する。

use std::sync::Arc;

use shiguredo_audio_device::{AudioCapture, AudioCaptureConfig};
use shiguredo_video_device::{VideoCapture, VideoCaptureConfig};
use shiguredo_webrtc::{
    AdaptFrameResult, AdaptedVideoTrackSource, AudioDeviceModule, I420Buffer, TimestampAligner,
};
use sora_sdk::{
    AdmConfig, Audio, CodecDirection, JsonString, ProxyInfo, Role, SoraConnection,
    SoraConnectionContext, SoraConnectionContextConfig, Video, VideoCodecPreference,
};
use tokio::sync::{mpsc, oneshot};
use tracing::info;

use crate::adm::{AdmState, build_adm_handler};
use crate::error::BoxError;
use crate::fake::start_fake_video_thread;
use crate::metrics::MetricsState;

// ─── 設定 ─────────────────────────────────────────────────────────────────────

#[allow(dead_code)]
pub struct SoraConfig {
    pub signaling_urls: Vec<String>,
    pub channel_id: String,
    pub role: Role,
    pub video: bool,
    pub audio: bool,
    pub video_codec_type: Option<String>,
    pub audio_codec_type: Option<String>,
    pub video_bit_rate: u32,
    pub audio_bit_rate: u32,
    pub h264_encoder: Option<String>,
    pub h264_decoder: Option<String>,
    pub h265_encoder: Option<String>,
    pub h265_decoder: Option<String>,
    pub spotlight: bool,
    pub simulcast: bool,
    pub data_channel_signaling: Option<bool>,
    pub ignore_disconnect_websocket: Option<bool>,
    pub metadata: Option<String>,
    pub no_audio_device: bool,
    pub no_video_input_device: bool,
    pub fake_capture_device: bool,
    pub use_v4l2_encoder: bool,
    pub openh264_lib: Option<shiguredo_openh264::Openh264Library>,
    pub use_libcamera: bool,
    pub use_libcamera_native: bool,
    pub libcamera_controls: Vec<(String, String)>,
    pub video_input_device: Option<String>,
    pub audio_input_device: Option<String>,
    pub video_width: i32,
    pub video_height: i32,
    pub framerate: u32,
    pub disconnect_wait_timeout: u32,
    pub insecure: bool,
    pub force_pixel_format: Option<shiguredo_video_device::PixelFormat>,
    /// クライアント証明書 (cert_pem, key_pem)
    pub client_cert: Option<(String, String)>,
    /// CA 証明書 (PEM)
    pub ca_cert: Option<String>,
    /// プロキシ URL
    pub proxy_url: Option<String>,
    /// プロキシ認証ユーザー名
    pub proxy_username: Option<String>,
    /// プロキシ認証パスワード
    pub proxy_password: Option<String>,
    /// フェイク音声のビープトリガー
    pub beep_trigger: Option<crate::fake::BeepTrigger>,
    /// プレビューフレーム送信チャネル
    #[cfg(feature = "player")]
    pub preview_tx: Option<std::sync::mpsc::SyncSender<crate::preview::PreviewFrame>>,
}

// ─── 公開 API ─────────────────────────────────────────────────────────────────

pub async fn run(
    config: SoraConfig,
    metrics_state: Option<Arc<MetricsState>>,
    fake_adm: Option<crate::fake::SendableAdm>,
) -> Result<(), BoxError> {
    // ── ADM 構築 ──────────────────────────────────────────────────────────
    let (adm_config, adm_state, audio_capture) = if let Some(sendable) = fake_adm {
        info!(target: "sora", "fake audio (--fake-capture-device)");
        (AdmConfig::UseExternal(sendable.0), None, None)
    } else {
        build_adm(&config)?
    };

    // ── SoraConnectionContext 生成 ────────────────────────────────────────
    // サイマルキャスト時はネイティブバッファを使用しない
    #[cfg(feature = "raspberrypi")]
    let dmabuf_map = if config.use_libcamera_native
        && config.use_libcamera
        && config.use_v4l2_encoder
        && !config.simulcast
    {
        Some(crate::libcamera::DmaBufMap::default())
    } else {
        None
    };

    // ctx_config は Send を実装しないため、await をまたがないようブロックで囲む
    let context = {
        #[allow(unused_mut)]
        let mut ctx_config = SoraConnectionContextConfig {
            adm_config,
            ..Default::default()
        };

        apply_video_toolbox_preference(
            &mut ctx_config.video_codec_preference,
            CodecDirection::Encoder,
            shiguredo_webrtc::VideoCodecType::H264,
            config.h264_encoder.as_deref(),
        )?;
        apply_video_toolbox_preference(
            &mut ctx_config.video_codec_preference,
            CodecDirection::Decoder,
            shiguredo_webrtc::VideoCodecType::H264,
            config.h264_decoder.as_deref(),
        )?;
        apply_video_toolbox_preference(
            &mut ctx_config.video_codec_preference,
            CodecDirection::Encoder,
            shiguredo_webrtc::VideoCodecType::H265,
            config.h265_encoder.as_deref(),
        )?;
        apply_video_toolbox_preference(
            &mut ctx_config.video_codec_preference,
            CodecDirection::Decoder,
            shiguredo_webrtc::VideoCodecType::H265,
            config.h265_decoder.as_deref(),
        )?;

        // V4L2 ハードウェアエンコーダーの追加
        if config.use_v4l2_encoder {
            #[cfg(feature = "raspberrypi")]
            {
                use crate::v4l2_encoder::sora_capability::V4l2VideoCodecCapability;
                info!(target: "sora", "using V4L2 H.264 hardware encoder capability");
                ctx_config
                    .video_codec_capabilities
                    .push(Box::new(V4l2VideoCodecCapability {
                        dmabuf_map: dmabuf_map.clone(),
                    }));
            }
            #[cfg(not(feature = "raspberrypi"))]
            {
                tracing::warn!(target: "sora", "--use-v4l2-encoder requires raspberrypi feature");
            }
        }
        // OpenH264 エンコーダー/デコーダーの追加
        if let Some(ref lib) = config.openh264_lib {
            use crate::openh264::sora_capability::Openh264VideoCodecCapability;
            info!(target: "sora", "using OpenH264 codec capability");
            ctx_config
                .video_codec_capabilities
                .push(Box::new(Openh264VideoCodecCapability { lib: lib.clone() }));
        }

        SoraConnectionContext::new_with_config(ctx_config).map_err(|e| -> BoxError {
            format!("SoraConnectionContext の生成に失敗: {e}").into()
        })?
    };

    // ── 映像キャプチャ・トラック ──────────────────────────────────────────
    #[cfg(feature = "raspberrypi")]
    let mut _libcamera_capture = None;

    let (video_track, _video_capture) = if config.role.wants_send()
        && config.video
        && !config.no_video_input_device
    {
        let source = AdaptedVideoTrackSource::new();
        let vts = source.cast_to_video_track_source();

        if config.use_libcamera {
            #[cfg(feature = "raspberrypi")]
            {
                let capture = crate::libcamera::start_libcamera_capture(
                    source,
                    config.video_width as u32,
                    config.video_height as u32,
                    config.libcamera_controls.clone(),
                    dmabuf_map.clone(),
                )?;
                _libcamera_capture = Some(capture);
                info!(target: "sora", width = config.video_width, height = config.video_height, "libcamera video started");
                let track = context.create_video_track(&vts).map_err(|e| -> BoxError {
                    format!("映像トラックの生成に失敗: {e}").into()
                })?;
                (Some(track), None)
            }
            #[cfg(not(feature = "raspberrypi"))]
            {
                let _ = (source, vts);
                return Err(
                    "--use-libcamera は raspberrypi feature が有効な場合のみ使用できます".into(),
                );
            }
        } else if config.fake_capture_device {
            start_fake_video_thread(
                source,
                config.video_width,
                config.video_height,
                config.framerate,
                config.beep_trigger.clone(),
                #[cfg(feature = "player")]
                config.preview_tx.clone(),
            );
            info!(target: "sora", width = config.video_width, height = config.video_height, fps = config.framerate, "fake video started");
            let track = context.create_video_track(&vts).map_err(|e| -> BoxError {
                format!("映像トラックの生成に失敗: {e}").into()
            })?;
            (Some(track), None)
        } else {
            let shared = Arc::new(std::sync::Mutex::new((source, TimestampAligner::new())));

            #[cfg(feature = "player")]
            let preview_tx = config.preview_tx.clone();

            let video_cfg = VideoCaptureConfig {
                device_id: config.video_input_device.clone(),
                width: config.video_width,
                height: config.video_height,
                fps: config.framerate as i32,
                pixel_format: config.force_pixel_format,
            };
            let mut capture = VideoCapture::new(video_cfg, move |frame| {
                let Some(buffer) = crate::webrtc_video::capture_frame_to_i420(
                    frame.pixel_format,
                    frame.data,
                    frame.stride,
                    frame.stride_uv,
                    frame.uv_data,
                    frame.width,
                    frame.height,
                ) else {
                    return;
                };
                let Ok(mut guard) = shared.lock() else { return };
                let (ref mut source, ref mut aligner) = *guard;

                let AdaptFrameResult { applied, size } =
                    source.adapt_frame(frame.width, frame.height, frame.timestamp_us);
                if !applied {
                    return;
                }

                let ts =
                    aligner.translate(frame.timestamp_us, shiguredo_webrtc::time_millis() * 1000);
                if size.adapted_width != frame.width || size.adapted_height != frame.height {
                    let mut scaled = I420Buffer::new(size.adapted_width, size.adapted_height);
                    scaled.scale_from(&buffer);

                    // プレビューウィンドウへフレームを送信 (ベストエフォート)
                    #[cfg(feature = "player")]
                    if let Some(ref tx) = preview_tx {
                        let pf = crate::preview::extract_preview_frame(
                            &scaled,
                            size.adapted_width,
                            size.adapted_height,
                            ts,
                        );
                        let _ = tx.try_send(pf);
                    }

                    let video_frame = crate::webrtc_video::video_frame_from_i420(
                        &scaled,
                        ts,
                        (ts * 90 / 1000) as u32,
                    );
                    source.on_frame(&video_frame);
                } else {
                    // プレビューウィンドウへフレームを送信 (ベストエフォート)
                    #[cfg(feature = "player")]
                    if let Some(ref tx) = preview_tx {
                        let pf = crate::preview::extract_preview_frame(
                            &buffer,
                            frame.width,
                            frame.height,
                            ts,
                        );
                        let _ = tx.try_send(pf);
                    }

                    let video_frame = crate::webrtc_video::video_frame_from_i420(
                        &buffer,
                        ts,
                        (ts * 90 / 1000) as u32,
                    );
                    source.on_frame(&video_frame);
                }
            })
            .map_err(|e| format!("映像キャプチャの初期化に失敗: {e}"))?;

            capture
                .start()
                .map_err(|e| format!("映像キャプチャの開始に失敗: {e}"))?;
            info!(target: "sora", "video capture started");

            let track = context.create_video_track(&vts).map_err(|e| -> BoxError {
                format!("映像トラックの生成に失敗: {e}").into()
            })?;
            (Some(track), Some(capture))
        }
    } else {
        if config.no_video_input_device {
            info!(target: "sora", "映像を無効化しました (--no-video-input-device)");
        }
        (None, None)
    };

    // ── 音声トラック ─────────────────────────────────────────────────────
    let audio_track = if config.role.wants_send() && config.audio {
        let audio_source = context.create_audio_source().map_err(|e| -> BoxError {
            format!("音声ソースの生成に失敗: {e}").into()
        })?;
        let track = context
            .create_audio_track(&audio_source)
            .map_err(|e| -> BoxError {
                format!("音声トラックの生成に失敗: {e}").into()
            })?;
        Some(track)
    } else {
        None
    };

    // ── SoraConnection ビルド ────────────────────────────────────────────
    let mut builder = SoraConnection::builder(
        context,
        config.signaling_urls.clone(),
        config.channel_id.clone(),
        config.role,
    );

    builder = builder
        .video(build_video(
            config.video,
            config.video_codec_type.as_deref(),
            config.video_bit_rate,
        )?)
        .audio(build_audio(
            config.audio,
            config.audio_codec_type.as_deref(),
            config.audio_bit_rate,
        )?);

    if let Some(track) = video_track {
        builder = builder.sender_video_track(track);
    }
    if let Some(track) = audio_track {
        builder = builder.sender_audio_track(track);
    }
    if let Some(dc) = config.data_channel_signaling {
        builder = builder.data_channel_signaling(dc);
    }
    if let Some(idw) = config.ignore_disconnect_websocket {
        builder = builder.ignore_disconnect_websocket(idw);
    }
    if config.simulcast {
        builder = builder.simulcast(true);
    }
    if config.spotlight {
        builder = builder.spotlight(true);
    }
    if let Some(ref metadata_str) = config.metadata {
        let metadata: JsonString = metadata_str.parse().map_err(|e| -> BoxError {
            format!("metadata の JSON パースに失敗: {e}").into()
        })?;
        builder = builder.metadata(metadata);
    }
    if config.disconnect_wait_timeout > 0 {
        builder = builder.disconnect_wait_timeout(std::time::Duration::from_secs(
            config.disconnect_wait_timeout as u64,
        ));
    }
    if config.insecure {
        builder = builder.insecure(true).turn_tls_insecure(true);
    }
    if let Some(ca_pem) = config.ca_cert {
        builder = builder.ca_cert(ca_pem);
    }
    if let Some((cert_pem, key_pem)) = config.client_cert {
        builder = builder.client_cert(cert_pem, key_pem);
    }
    if let Some(proxy_url) = config.proxy_url {
        builder = builder.proxy(ProxyInfo {
            url: proxy_url,
            username: config.proxy_username,
            password: config.proxy_password,
            user_agent: None,
        });
    }

    builder = builder
        .on_notify(|msg| {
            info!(target: "sora", notify = %msg, "on_notify");
        })
        .on_track(|_transceiver| {
            info!(target: "sora", "on_track");
        });

    let (client, handle) = builder
        .build()
        .map_err(|e| -> BoxError { format!("SoraConnection のビルドに失敗: {e}").into() })?;

    // ── メトリクス stats プロバイダー登録 ─────────────────────────────────
    if let Some(ref ms) = metrics_state {
        let (stats_tx, mut stats_rx) = mpsc::channel::<oneshot::Sender<String>>(1);
        ms.register(stats_tx).await;

        let stats_handle = handle.clone();
        tokio::spawn(async move {
            while let Some(reply_tx) = stats_rx.recv().await {
                match stats_handle.get_stats().await {
                    Ok(json) => {
                        let _ = reply_tx.send(json.to_string());
                    }
                    Err(_) => {
                        let _ = reply_tx.send("[]".to_string());
                    }
                }
            }
        });
    }

    info!(
        target: "sora",
        signaling_urls = ?config.signaling_urls,
        channel_id = %config.channel_id,
        "connecting to Sora"
    );

    // ── 実行 ─────────────────────────────────────────────────────────────
    client
        .run()
        .await
        .map_err(|e| -> BoxError { format!("Sora client error: {e}").into() })?;

    // キャプチャリソースの Drop 順序を明示
    drop(_video_capture);
    #[cfg(feature = "raspberrypi")]
    drop(_libcamera_capture);
    drop(audio_capture);
    drop(adm_state);

    Ok(())
}

fn apply_video_toolbox_preference(
    preference: &mut VideoCodecPreference,
    direction: CodecDirection,
    codec_type: shiguredo_webrtc::VideoCodecType,
    value: Option<&str>,
) -> Result<(), BoxError> {
    let Some(value) = value else {
        return Ok(());
    };
    if value != "videotoolbox" {
        return Ok(());
    }

    #[cfg(not(any(target_os = "macos", target_os = "ios")))]
    {
        // Apple プラットフォーム以外では利用不可
        let _ = (preference, direction, codec_type);
        Err("videotoolbox is only available on Apple platforms".into())
    }

    #[cfg(any(target_os = "macos", target_os = "ios"))]
    {
        let Some(codec) = preference.find_mut(direction, codec_type) else {
            return Err(format!(
                "video codec preference not found: direction={direction:?}, codec_type={codec_type:?}"
            )
            .into());
        };
        codec.set_implementation(sora_sdk::VideoCodecImplementation::new(
            "internal-hwa",
            "WebRTC ObjC default VideoCodecFactory",
        ));
        Ok(())
    }
}

// ─── ADM 構築 ─────────────────────────────────────────────────────────────────

fn build_adm(
    config: &SoraConfig,
) -> Result<(AdmConfig, Option<AdmState>, Option<AudioCapture>), BoxError> {
    if config.no_audio_device || config.fake_capture_device {
        if config.fake_capture_device {
            info!(target: "sora", "fake audio handled by FakeAudioCapturer");
        } else {
            info!(target: "sora", "音声を無効化しました (--no-audio-device)");
        }
        Ok((AdmConfig::NoAudioDevice, None, None))
    } else {
        let state = AdmState::new();
        let handler = build_adm_handler(&state);
        let adm = AudioDeviceModule::new_with_handler(handler);

        let state_for_cap = state.clone();
        let audio_cfg = AudioCaptureConfig {
            device_id: config.audio_input_device.clone(),
            ..Default::default()
        };
        let mut capture = AudioCapture::new(audio_cfg, move |frame| {
            state_for_cap.on_audio_frame(&frame);
        })
        .map_err(|e| format!("音声キャプチャの初期化に失敗: {e}"))?;

        capture
            .start()
            .map_err(|e| format!("音声キャプチャの開始に失敗: {e}"))?;
        info!(target: "sora", "audio capture started");

        Ok((AdmConfig::UseExternal(adm), Some(state), Some(capture)))
    }
}

// ─── Video/Audio ヘルパー ─────────────────────────────────────────────────────

fn build_video(enabled: bool, codec_type: Option<&str>, bit_rate: u32) -> Result<Video, BoxError> {
    if !enabled {
        return Ok(Video::new_bool(false));
    }
    let br = if bit_rate > 0 { Some(bit_rate) } else { None };
    match codec_type {
        None => Ok(Video::new_bool(true)),
        Some("VP8") => Ok(Video::new_vp8(br)),
        Some("VP9") => Ok(Video::new_vp9(br, None)),
        Some("AV1") => Ok(Video::new_av1(br, None)),
        Some("H264") => Ok(Video::new_h264(br, None)),
        Some("H265") => Ok(Video::new_h265(br, None)),
        Some(other) => Err(format!(
            "不正な video-codec-type: {other} (VP8, VP9, AV1, H264, H265 のいずれかを指定してください)"
        )
        .into()),
    }
}

fn build_audio(enabled: bool, codec_type: Option<&str>, bit_rate: u32) -> Result<Audio, BoxError> {
    if !enabled {
        return Ok(Audio::new_bool(false));
    }
    let br = if bit_rate > 0 { Some(bit_rate) } else { None };
    match codec_type {
        None => Ok(Audio::new_bool(true)),
        Some("OPUS") => Ok(Audio::new_opus(br, None)),
        Some(other) => {
            Err(format!("不正な audio-codec-type: {other} (OPUS のみ対応しています)").into())
        }
    }
}
