use std::sync::{Arc, Mutex};

use shiguredo_audio_device::{AudioCapture, AudioCaptureConfig};

/// oneshot チャンネルの送信側を保護するための型エイリアス
type SharedSender<T> = Arc<Mutex<Option<oneshot::Sender<T>>>>;
use shiguredo_video_device::{VideoCapture, VideoCaptureConfig};
use shiguredo_webrtc::{
    AdaptFrameResult, AdaptedVideoTrackSource, AudioDecoderFactory, AudioDeviceModule,
    AudioEncoderFactory, AudioProcessingBuilder, CreateSessionDescriptionObserver,
    CreateSessionDescriptionObserverHandler, DataChannel, Environment, I420Buffer, IceServer,
    MediaType, PeerConnection, PeerConnectionDependencies, PeerConnectionFactoryDependencies,
    PeerConnectionObserver, PeerConnectionObserverHandler, PeerConnectionOfferAnswerOptions,
    PeerConnectionRtcConfiguration, PeerConnectionState, RtcError, RtpTransceiverInit, SdpType,
    SessionDescription, SetLocalDescriptionObserver, SetLocalDescriptionObserverHandler,
    SetRemoteDescriptionObserver, SetRemoteDescriptionObserverHandler, Thread, TimestampAligner,
    VideoDecoderFactory, VideoEncoderFactory, VideoTrackSource,
};
use tokio::sync::{mpsc, oneshot};
use tracing::{info, warn};

use super::P2PConfig;
use super::message::{json_answer, json_candidate};
use crate::adm::{AdmState, build_adm_handler};
use crate::error::{BoxError, wrtc_err};

// ─── WebRTC エンジン ────────────────────────────────────────────────────────

/// WebRTC エンジン（PeerConnectionFactory + C++ スレッド群）
///
/// フィールドは宣言順に drop される。
/// キャプチャを先に drop し、その後 factory、最後にスレッド群を drop する。
pub(super) struct WebRtcEngine {
    // 1. キャプチャは factory より先に止める
    _video_capture: Option<VideoCapture>,
    #[cfg(feature = "raspberrypi")]
    _libcamera_capture: Option<crate::libcamera::LibcameraCapture>,
    _audio_capture: Option<AudioCapture>,
    // 2. トラックソースと ADM 状態
    pub(super) video_track_source: Option<VideoTrackSource>,
    _adm_state: Option<AdmState>,
    // 3. factory と設定
    pub(super) factory: shiguredo_webrtc::PeerConnectionFactory,
    pub(super) degradation_preference: shiguredo_webrtc::DegradationPreference,
    pub(super) video_codec_type: Option<String>,
    pub(super) audio_codec_type: Option<String>,
    // 4. スレッド群（factory より後に drop）
    _env: Environment,
    _network_thread: Thread,
    _worker_thread: Thread,
    _signaling_thread: Thread,
}

impl WebRtcEngine {
    pub(super) fn new(config: &P2PConfig) -> Result<Self, BoxError> {
        let env = Environment::new();
        let mut network_thread = Thread::new_with_socket_server();
        let mut worker_thread = Thread::new();
        let mut signaling_thread = Thread::new();
        network_thread.start();
        worker_thread.start();
        signaling_thread.start();

        // ── 映像デバイス ──────────────────────────────────────────────────
        #[cfg(feature = "raspberrypi")]
        let mut libcamera_capture = None;
        // ネイティブモード: libcamera → V4L2 エンコーダー間の DMA-BUF 受け渡し
        #[cfg(feature = "raspberrypi")]
        let dmabuf_map =
            if config.use_libcamera_native && config.use_libcamera && config.use_v4l2_encoder {
                Some(crate::libcamera::DmaBufMap::default())
            } else {
                None
            };

        let (video_track_source, video_capture) = if !config.no_video_input_device {
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
                    libcamera_capture = Some(capture);
                    info!(target: "dev", width = config.video_width, height = config.video_height, "libcamera video started");
                    (Some(vts), None)
                }
                #[cfg(not(feature = "raspberrypi"))]
                {
                    let _ = (source, vts);
                    return Err(
                        "--use-libcamera は raspberrypi feature が有効な場合のみ使用できます"
                            .into(),
                    );
                }
            } else if config.fake_capture_device {
                crate::fake::start_fake_video_thread(
                    source,
                    config.video_width,
                    config.video_height,
                    config.framerate,
                    #[cfg(feature = "preview")]
                    None,
                );
                info!(target: "dev", width = config.video_width, height = config.video_height, fps = config.framerate, "fake video started");
                (Some(vts), None)
            } else {
                let shared = Arc::new(std::sync::Mutex::new((source, TimestampAligner::new())));

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

                    let ts = aligner
                        .translate(frame.timestamp_us, shiguredo_webrtc::time_millis() * 1000);
                    let video_frame = if size.adapted_width != frame.width
                        || size.adapted_height != frame.height
                    {
                        let mut scaled = I420Buffer::new(size.adapted_width, size.adapted_height);
                        scaled.scale_from(&buffer);
                        crate::webrtc_video::video_frame_from_i420(
                            &scaled,
                            ts,
                            (ts * 90 / 1000) as u32,
                        )
                    } else {
                        crate::webrtc_video::video_frame_from_i420(
                            &buffer,
                            ts,
                            (ts * 90 / 1000) as u32,
                        )
                    };
                    source.on_frame(&video_frame);
                })
                .map_err(|e| format!("映像キャプチャの初期化に失敗: {e}"))?;

                capture
                    .start()
                    .map_err(|e| format!("映像キャプチャの開始に失敗: {e}"))?;
                info!(target: "dev", "video capture started");

                (Some(vts), Some(capture))
            }
        } else {
            info!(target: "dev", "video input device disabled (--no-video-input-device)");
            (None, None)
        };

        // ── 音声デバイス ──────────────────────────────────────────────────
        let (adm_state, audio_capture, adm_opt) =
            if !config.no_audio_device && !config.fake_capture_device {
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
                info!(target: "dev", "audio capture started");

                (Some(state), Some(capture), Some(adm))
            } else if config.fake_capture_device && !config.no_audio_device {
                info!(target: "dev", "fake audio (--fake-capture-device)");
                let adm = crate::fake::create_dummy_adm(&env)?;
                (None, None, Some(adm))
            } else {
                info!(target: "dev", "audio device disabled (--no-audio-device)");
                (None, None, None)
            };

        // ── PeerConnectionFactory ─────────────────────────────────────────
        let mut deps = PeerConnectionFactoryDependencies::new();
        deps.set_network_thread(&network_thread);
        deps.set_worker_thread(&worker_thread);
        deps.set_signaling_thread(&signaling_thread);
        deps.set_audio_encoder_factory(&AudioEncoderFactory::builtin());
        deps.set_audio_decoder_factory(&AudioDecoderFactory::builtin());
        deps.set_audio_processing_builder(AudioProcessingBuilder::new_builtin());

        // 映像エンコーダーファクトリの選択
        if config.use_v4l2_encoder {
            #[cfg(feature = "raspberrypi")]
            {
                info!(target: "dev", "using V4L2 H.264 hardware encoder factory");
                deps.set_video_encoder_factory(crate::v4l2_encoder::create_v4l2_encoder_factory(
                    dmabuf_map.clone(),
                ));
            }
            #[cfg(not(feature = "raspberrypi"))]
            {
                warn!(target: "dev", "--use-v4l2-encoder requires raspberrypi feature, falling back to builtin");
                deps.set_video_encoder_factory(VideoEncoderFactory::builtin());
            }
        } else if let Some(ref lib) = config.openh264_lib {
            info!(target: "dev", "using OpenH264 encoder factory");
            deps.set_video_encoder_factory(crate::openh264::create_openh264_encoder_factory(lib));
        } else {
            deps.set_video_encoder_factory(VideoEncoderFactory::builtin());
        }

        // 映像デコーダーファクトリの選択
        if let Some(ref lib) = config.openh264_lib {
            info!(target: "dev", "using OpenH264 decoder factory");
            deps.set_video_decoder_factory(crate::openh264::create_openh264_decoder_factory(lib));
        } else {
            deps.set_video_decoder_factory(VideoDecoderFactory::builtin());
        }
        if let Some(ref adm) = adm_opt {
            deps.set_audio_device_module(adm);
        }
        deps.enable_media();

        let factory =
            shiguredo_webrtc::PeerConnectionFactory::create_modular(&mut deps).map_err(wrtc_err)?;

        Ok(Self {
            _video_capture: video_capture,
            #[cfg(feature = "raspberrypi")]
            _libcamera_capture: libcamera_capture,
            _audio_capture: audio_capture,
            video_track_source,
            _adm_state: adm_state,
            factory,
            degradation_preference: config.degradation_preference,
            video_codec_type: config.video_codec_type.clone(),
            audio_codec_type: config.audio_codec_type.clone(),
            _env: env,
            _network_thread: network_thread,
            _worker_thread: worker_thread,
            _signaling_thread: signaling_thread,
        })
    }
}

// PeerConnectionFactory は Send + Sync なので WebRtcEngine も Arc で共有できる
unsafe impl Send for WebRtcEngine {}
unsafe impl Sync for WebRtcEngine {}

// ─── ピア接続 ──────────────────────────────────────────────────────────────

/// ピア接続（PeerConnection + Observer のライフタイム管理）
///
/// フィールドは宣言順に drop される。
/// pc が先に drop されることで、Observer へのコールバックが止まってから Observer が解放される。
pub(super) struct Peer {
    pub(super) pc: PeerConnection,
    _observer: PeerConnectionObserver,
}

/// PeerConnection と Observer を作成して Peer を返す
///
/// 映像トラックソースがある場合は PeerConnection に追加する。
#[cfg(target_os = "linux")]
pub(super) fn create_peer(
    engine: &WebRtcEngine,
    sig_tx: mpsc::UnboundedSender<String>,
    serial_config: Option<crate::serial::SerialConfig>,
) -> Result<Peer, BoxError> {
    create_peer_inner(engine, sig_tx, serial_config)
}

/// PeerConnection と Observer を作成して Peer を返す
///
/// 映像トラックソースがある場合は PeerConnection に追加する。
#[cfg(not(target_os = "linux"))]
pub(super) fn create_peer(
    engine: &WebRtcEngine,
    sig_tx: mpsc::UnboundedSender<String>,
) -> Result<Peer, BoxError> {
    create_peer_inner(engine, sig_tx)
}

fn create_peer_inner(
    engine: &WebRtcEngine,
    sig_tx: mpsc::UnboundedSender<String>,
    #[cfg(target_os = "linux")] serial_config: Option<crate::serial::SerialConfig>,
) -> Result<Peer, BoxError> {
    struct PcObserver {
        sig_tx: mpsc::UnboundedSender<String>,
        #[cfg(target_os = "linux")]
        serial_config: Option<crate::serial::SerialConfig>,
    }
    impl PeerConnectionObserverHandler for PcObserver {
        fn on_connection_change(&mut self, state: PeerConnectionState) {
            info!(target: "pc", state = ?state, "state changed");
        }
        fn on_ice_candidate(&mut self, candidate: shiguredo_webrtc::IceCandidateRef<'_>) {
            let sdp = match candidate.to_string() {
                Ok(s) => s,
                Err(e) => {
                    warn!(target: "sdp", error = ?e, "ICE candidate to_string error");
                    return;
                }
            };
            let sdp_mid = candidate.sdp_mid().unwrap_or_default();
            let sdp_mline_index = candidate.sdp_mline_index();
            let msg = json_candidate(&sdp, &sdp_mid, sdp_mline_index);
            let _ = self.sig_tx.send(msg);
        }
        fn on_data_channel(&mut self, dc: DataChannel) {
            let label = dc.label().unwrap_or_default();
            info!(target: "dc", label = %label, "DataChannel received");
            #[cfg(target_os = "linux")]
            if label == "serial" {
                if let Some(ref config) = self.serial_config {
                    crate::serial::start_serial_bridge(config, dc);
                } else {
                    warn!(target: "dc", "serial DataChannel received but --serial not configured");
                }
            }
        }
    }

    let observer = PeerConnectionObserver::new_with_handler(Box::new(PcObserver {
        sig_tx,
        #[cfg(target_os = "linux")]
        serial_config,
    }));

    let mut rtc_config = PeerConnectionRtcConfiguration::new();
    {
        let mut ice_server = IceServer::new();
        ice_server.add_url("stun:stun.l.google.com:19302");
        rtc_config.servers().push(&ice_server);
    }

    let mut deps = PeerConnectionDependencies::new(&observer);
    let pc =
        PeerConnection::create(&engine.factory, &mut rtc_config, &mut deps).map_err(wrtc_err)?;

    // 音声トランシーバーを追加
    let mut audio_transceiver = {
        let mut init = RtpTransceiverInit::new();
        pc.add_transceiver(MediaType::Audio, &mut init)
            .map_err(wrtc_err)?
    };

    // 映像トラックを追加（ブラウザが recvonly で受信する側）
    let mut video_transceiver = if let Some(ref vts) = engine.video_track_source {
        let video_track = engine
            .factory
            .create_video_track(vts, "video0")
            .map_err(wrtc_err)?;
        let mut init = RtpTransceiverInit::new();
        let transceiver = pc
            .add_transceiver_with_track(&video_track, &mut init)
            .map_err(wrtc_err)?;
        info!(target: "pc", degradation = ?engine.degradation_preference, "video track added");
        Some(transceiver)
    } else {
        None
    };

    // コーデックプリファレンスを設定
    if engine.video_codec_type.is_some() || engine.audio_codec_type.is_some() {
        crate::codec::set_codec_preferences(
            &engine.factory,
            &mut audio_transceiver,
            video_transceiver.as_mut(),
            engine.video_codec_type.as_deref(),
            engine.audio_codec_type.as_deref(),
        )?;
    }

    Ok(Peer {
        pc,
        _observer: observer,
    })
}

// ─── SDP 非同期ラッパー ────────────────────────────────────────────────────

/// SetRemoteDescription を非同期で呼び出す
///
/// `SessionDescription` は `!Send` なので async fn にできない。
/// 同期部分で `desc` を消費し、`Send` な型のみを async move ブロックに渡す。
pub(super) fn set_remote_desc(
    pc: &PeerConnection,
    desc: SessionDescription,
) -> impl std::future::Future<Output = Result<(), crate::error::BoxError>> + Send {
    let (tx, rx) = oneshot::channel::<Result<(), String>>();
    let tx = Arc::new(Mutex::new(Some(tx)));

    struct SrdHandler {
        tx: SharedSender<Result<(), String>>,
    }
    impl SetRemoteDescriptionObserverHandler for SrdHandler {
        fn on_set_remote_description_complete(&mut self, error: RtcError) {
            if let Ok(mut guard) = self.tx.lock()
                && let Some(t) = guard.take()
            {
                let r = if error.ok() {
                    Ok(())
                } else {
                    Err(error
                        .message()
                        .unwrap_or_else(|_| "SetRemoteDescription エラー".into()))
                };
                let _ = t.send(r);
            }
        }
    }

    let observer =
        SetRemoteDescriptionObserver::new_with_handler(Box::new(SrdHandler { tx: tx.clone() }));

    // desc (!Send) はここで同期的に消費する
    pc.set_remote_description(desc, &observer);

    // observer (Send: unsafe impl Send) と rx (Send) のみを async move に渡す
    async move {
        let result = rx
            .await
            .map_err(|_| "SetRemoteDescription チャンネルクローズ")?;
        drop(observer);
        result.map_err(Into::into)
    }
}

/// CreateAnswer を非同期で呼び出し SDP 文字列を返す
///
/// `PeerConnectionOfferAnswerOptions` は `!Send` なので async fn にできない。
pub(super) fn create_answer(
    pc: &PeerConnection,
) -> impl std::future::Future<Output = Result<String, crate::error::BoxError>> + Send {
    let (tx, rx) = oneshot::channel::<Result<String, String>>();
    let tx = Arc::new(Mutex::new(Some(tx)));

    struct CsdHandler {
        tx: SharedSender<Result<String, String>>,
    }
    impl CreateSessionDescriptionObserverHandler for CsdHandler {
        fn on_success(&mut self, desc: SessionDescription) {
            if let Ok(mut guard) = self.tx.lock()
                && let Some(t) = guard.take()
            {
                let r = desc
                    .to_string()
                    .map_err(|e| format!("SDP to_string: {e:?}"));
                let _ = t.send(r);
            }
        }
        fn on_failure(&mut self, error: RtcError) {
            if let Ok(mut guard) = self.tx.lock()
                && let Some(t) = guard.take()
            {
                let msg = error
                    .message()
                    .unwrap_or_else(|_| "CreateAnswer エラー".into());
                let _ = t.send(Err(msg));
            }
        }
    }

    let mut observer =
        CreateSessionDescriptionObserver::new_with_handler(Box::new(CsdHandler { tx: tx.clone() }));

    // options (!Send) はブロック内で消費してから observer を async move に渡す
    {
        let mut options = PeerConnectionOfferAnswerOptions::new();
        pc.create_answer(&mut observer, &mut options);
    }

    // observer (Send: unsafe impl Send) と rx (Send) のみを async move に渡す
    async move {
        let result = rx
            .await
            .map_err(|_| "CreateAnswer チャンネルクローズ")?
            .map_err(Into::into);
        drop(observer);
        result
    }
}

/// SetLocalDescription を非同期で呼び出す
///
/// `SessionDescription` は `!Send` なので async fn にできない。
pub(super) fn set_local_desc(
    pc: &PeerConnection,
    desc: SessionDescription,
) -> impl std::future::Future<Output = Result<(), crate::error::BoxError>> + Send {
    let (tx, rx) = oneshot::channel::<Result<(), String>>();
    let tx = Arc::new(Mutex::new(Some(tx)));

    struct SldHandler {
        tx: SharedSender<Result<(), String>>,
    }
    impl SetLocalDescriptionObserverHandler for SldHandler {
        fn on_set_local_description_complete(&mut self, error: RtcError) {
            if let Ok(mut guard) = self.tx.lock()
                && let Some(t) = guard.take()
            {
                let r = if error.ok() {
                    Ok(())
                } else {
                    Err(error
                        .message()
                        .unwrap_or_else(|_| "SetLocalDescription エラー".into()))
                };
                let _ = t.send(r);
            }
        }
    }

    let observer =
        SetLocalDescriptionObserver::new_with_handler(Box::new(SldHandler { tx: tx.clone() }));

    // desc (!Send) はここで同期的に消費する
    pc.set_local_description(desc, &observer);

    // observer (Send: unsafe impl Send) と rx (Send) のみを async move に渡す
    async move {
        let result = rx
            .await
            .map_err(|_| "SetLocalDescription チャンネルクローズ")?;
        drop(observer);
        result.map_err(Into::into)
    }
}

// ─── SDP 交換 ─────────────────────────────────────────────────────────────

/// offer を受け取り answer を生成して sig_tx に送信する
pub(super) async fn process_offer(
    pc: &PeerConnection,
    offer_sdp: String,
    sig_tx: &mpsc::UnboundedSender<String>,
) -> Result<(), crate::error::BoxError> {
    // 1. SetRemoteDescription (offer)
    let remote_desc = SessionDescription::new(SdpType::Offer, &offer_sdp).map_err(wrtc_err)?;
    set_remote_desc(pc, remote_desc).await?;

    // 2. CreateAnswer
    let answer_sdp = create_answer(pc).await?;

    // 3. SetLocalDescription (answer)
    let local_desc = SessionDescription::new(SdpType::Answer, &answer_sdp).map_err(wrtc_err)?;
    set_local_desc(pc, local_desc).await?;

    // 4. answer を送信
    let _ = sig_tx.send(json_answer(&answer_sdp));
    info!(target: "sdp", "answer sent");

    Ok(())
}
