use std::sync::{Arc, Mutex};

use shiguredo_audio_device::{AudioCapture, AudioCaptureConfig};

/// oneshot チャンネルの送信側を保護するための型エイリアス
type SharedSender<T> = Arc<Mutex<Option<tokio::sync::oneshot::Sender<T>>>>;
use shiguredo_video_device::{PixelFormat, VideoCapture, VideoCaptureConfig};
use shiguredo_webrtc::{
    AdaptFrameResult, AdaptedVideoTrackSource, AudioDecoderFactory, AudioDeviceModule,
    AudioEncoderFactory, AudioProcessingBuilder, CreateSessionDescriptionObserver,
    CreateSessionDescriptionObserverHandler, DataChannel, Environment, I420Buffer, IceServer,
    MediaType, PeerConnection, PeerConnectionDependencies, PeerConnectionFactoryDependencies,
    PeerConnectionObserver, PeerConnectionObserverHandler, PeerConnectionOfferAnswerOptions,
    PeerConnectionRtcConfiguration, PeerConnectionState, RtcError, RtpTransceiverDirection,
    RtpTransceiverInit, SdpType, SessionDescription, SetLocalDescriptionObserver,
    SetLocalDescriptionObserverHandler, SetRemoteDescriptionObserver,
    SetRemoteDescriptionObserverHandler, StringVector, Thread, TimestampAligner,
    VideoDecoderFactory, VideoEncoderFactory, VideoFrame as WebrtcVideoFrame, VideoTrackSource,
};
use tracing::info;

use super::message::{AcceptMessage, CandidateMessage, SdpMessage};
use super::signaling::SignalingCommand;
use super::{AyameConfig, Direction};
use crate::adm::{AdmState, build_adm_handler};
use crate::error::{BoxError, wrtc_err};
use crate::fake::start_fake_video_thread;

// ─── WebRTC エンジン ──────────────────────────────────────────────────────────

/// WebRTC エンジン (PeerConnectionFactory + C++ スレッド群)
///
/// フィールドは宣言順に drop される。
/// キャプチャを先に drop し、その後 factory、最後にスレッド群を drop する。
pub(super) struct AyameEngine {
    // 1. キャプチャは factory より先に止める
    _video_capture: Option<VideoCapture>,
    #[cfg(feature = "raspberrypi")]
    _libcamera_capture: Option<crate::libcamera::LibcameraCapture>,
    _audio_capture: Option<AudioCapture>,
    // 2. トラックソースと ADM 状態
    pub(super) video_track_source: Option<VideoTrackSource>,
    _adm_state: Option<AdmState>,
    // 3. factory
    pub(super) factory: shiguredo_webrtc::PeerConnectionFactory,
    // 4. スレッド群 (factory より後に drop)
    _env: Environment,
    _network_thread: Thread,
    _worker_thread: Thread,
    _signaling_thread: Thread,
}

unsafe impl Send for AyameEngine {}
unsafe impl Sync for AyameEngine {}

impl AyameEngine {
    pub(super) fn new(config: &AyameConfig) -> Result<Self, BoxError> {
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
                    info!(target: "ayame", width = config.video_width, height = config.video_height, "libcamera video started");
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
                start_fake_video_thread(
                    source,
                    config.video_width,
                    config.video_height,
                    config.framerate,
                    #[cfg(feature = "preview")]
                    None,
                );
                info!(target: "ayame", width = config.video_width, height = config.video_height, fps = config.framerate, "fake video started");
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
                    let i420 = match frame.pixel_format {
                        PixelFormat::Nv12 => {
                            let uv = frame.uv_data.unwrap_or(&[]);
                            shiguredo_webrtc::nv12_to_i420(
                                frame.data,
                                frame.stride,
                                uv,
                                frame.stride_uv,
                                frame.width,
                                frame.height,
                            )
                        }
                        PixelFormat::Yuy2 => shiguredo_webrtc::yuy2_to_i420(
                            frame.data,
                            frame.stride,
                            frame.width,
                            frame.height,
                        ),
                        _ => None,
                    };
                    let Some(buffer) = i420 else { return };
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
                        WebrtcVideoFrame::from_i420(&scaled, ts, (ts * 90 / 1000) as u32)
                    } else {
                        WebrtcVideoFrame::from_i420(&buffer, ts, (ts * 90 / 1000) as u32)
                    };
                    source.on_frame(&video_frame);
                })
                .map_err(|e| format!("映像キャプチャの初期化に失敗: {e}"))?;

                capture
                    .start()
                    .map_err(|e| format!("映像キャプチャの開始に失敗: {e}"))?;
                info!(target: "ayame", "video capture started");

                (Some(vts), Some(capture))
            }
        } else {
            info!(target: "ayame", "映像を無効化しました (--no-video-input-device)");
            (None, None)
        };

        // ── 音声デバイス ──────────────────────────────────────────────────
        let (adm_state, audio_capture, adm) =
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
                info!(target: "ayame", "audio capture started");

                (Some(state), Some(capture), adm)
            } else {
                if config.fake_capture_device {
                    info!(target: "ayame", "fake audio (--fake-capture-device)");
                } else {
                    info!(target: "ayame", "音声を無効化しました (--no-audio-device)");
                }
                let adm = crate::fake::create_dummy_adm(&env)?;
                (None, None, adm)
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
                info!(target: "ayame", "using V4L2 H.264 hardware encoder factory");
                deps.set_video_encoder_factory(crate::v4l2_encoder::create_v4l2_encoder_factory(
                    dmabuf_map.clone(),
                ));
            }
            #[cfg(not(feature = "raspberrypi"))]
            {
                tracing::warn!(target: "ayame", "--use-v4l2-encoder requires raspberrypi feature, falling back to builtin");
                deps.set_video_encoder_factory(VideoEncoderFactory::builtin());
            }
        } else if let Some(ref lib) = config.openh264_lib {
            info!(target: "ayame", "using OpenH264 encoder factory");
            deps.set_video_encoder_factory(crate::openh264::create_openh264_encoder_factory(lib));
        } else {
            deps.set_video_encoder_factory(VideoEncoderFactory::builtin());
        }

        // 映像デコーダーファクトリの選択
        if let Some(ref lib) = config.openh264_lib {
            info!(target: "ayame", "using OpenH264 decoder factory");
            deps.set_video_decoder_factory(crate::openh264::create_openh264_decoder_factory(lib));
        } else {
            deps.set_video_decoder_factory(VideoDecoderFactory::builtin());
        }
        deps.set_audio_device_module(&adm);
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
            _env: env,
            _network_thread: network_thread,
            _worker_thread: worker_thread,
            _signaling_thread: signaling_thread,
        })
    }
}

// ─── ピア接続 ─────────────────────────────────────────────────────────────────

/// ピア接続（PeerConnection + Observer のライフタイム管理）
///
/// フィールドは宣言順に drop される。
/// pc が先に drop されることで、Observer へのコールバックが止まってから Observer が解放される。
pub(super) struct Peer {
    pub(super) pc: PeerConnection,
    _observer: PeerConnectionObserver,
}

/// PeerConnection と Observer を作成して Peer を返す
pub(super) fn create_peer(
    engine: &AyameEngine,
    cmd_tx: tokio::sync::mpsc::Sender<SignalingCommand>,
    accept: &AcceptMessage,
    no_google_stun: bool,
    #[cfg(target_os = "linux")] serial_config: Option<crate::serial::SerialConfig>,
) -> Result<Peer, BoxError> {
    struct PcObserver {
        cmd_tx: tokio::sync::mpsc::Sender<SignalingCommand>,
        #[cfg(target_os = "linux")]
        serial_config: Option<crate::serial::SerialConfig>,
    }
    impl PeerConnectionObserverHandler for PcObserver {
        fn on_connection_change(&mut self, state: PeerConnectionState) {
            info!(target: "pc", state = ?state, "state changed");
        }
        fn on_ice_candidate(&mut self, candidate: shiguredo_webrtc::IceCandidateRef<'_>) {
            let sdp = candidate.to_string().unwrap_or_default();
            let sdp_mid = candidate.sdp_mid().unwrap_or_default();
            let sdp_mline_index = candidate.sdp_mline_index();
            let msg = CandidateMessage {
                candidate: &sdp,
                sdp_mid: &sdp_mid,
                sdp_mline_index,
            };
            // ICE コールバックは C++ スレッドから呼ばれるため blocking_send を使用
            let _ = self
                .cmd_tx
                .blocking_send(SignalingCommand::SendText(nojson::Json(&msg).to_string()));
        }
        fn on_data_channel(&mut self, dc: DataChannel) {
            let label = dc.label().unwrap_or_default();
            info!(target: "dc", label = %label, "DataChannel received");
            #[cfg(target_os = "linux")]
            if label == "serial" {
                if let Some(ref config) = self.serial_config {
                    crate::serial::start_serial_bridge(config, dc);
                } else {
                    tracing::warn!(target: "dc", "serial DataChannel received but --serial not configured");
                }
            }
        }
    }

    let observer = PeerConnectionObserver::new_with_handler(Box::new(PcObserver {
        cmd_tx,
        #[cfg(target_os = "linux")]
        serial_config,
    }));

    let mut rtc_config = PeerConnectionRtcConfiguration::new();
    configure_ice_servers(accept, &mut rtc_config, no_google_stun);

    let mut deps = PeerConnectionDependencies::new(&observer);
    let pc =
        PeerConnection::create(&engine.factory, &mut rtc_config, &mut deps).map_err(wrtc_err)?;

    Ok(Peer {
        pc,
        _observer: observer,
    })
}

// ─── ICE サーバー設定 ─────────────────────────────────────────────────────────

fn configure_ice_servers(
    accept: &AcceptMessage,
    config: &mut PeerConnectionRtcConfiguration,
    no_google_stun: bool,
) {
    let mut servers = config.servers();
    if !accept.ice_servers.is_empty() {
        for ice_cfg in &accept.ice_servers {
            let mut server = IceServer::new();
            for url in &ice_cfg.urls {
                server.add_url(url);
            }
            if let Some(u) = &ice_cfg.username {
                server.set_username(u);
            }
            if let Some(c) = &ice_cfg.credential {
                server.set_password(c);
            }
            servers.push(&server);
        }
    } else if !no_google_stun {
        let mut server = IceServer::new();
        server.add_url("stun:stun.l.google.com:19302");
        servers.push(&server);
    }
}

// ─── トランシーバー追加 ───────────────────────────────────────────────────────

fn rtp_direction(dir: &Direction) -> RtpTransceiverDirection {
    match dir {
        Direction::SendRecv => RtpTransceiverDirection::SendRecv,
        Direction::SendOnly => RtpTransceiverDirection::SendOnly,
        Direction::RecvOnly => RtpTransceiverDirection::RecvOnly,
    }
}

pub(super) fn sends_video(dir: &Direction) -> bool {
    matches!(dir, Direction::SendRecv | Direction::SendOnly)
}

pub(super) fn add_transceivers(
    pc: &PeerConnection,
    factory: &shiguredo_webrtc::PeerConnectionFactory,
    direction: &Direction,
    video_source: Option<&VideoTrackSource>,
    degradation_preference: shiguredo_webrtc::DegradationPreference,
) -> Result<(), BoxError> {
    // 音声トランシーバー
    {
        let mut init = RtpTransceiverInit::new();
        init.set_direction(rtp_direction(direction));
        pc.add_transceiver(MediaType::Audio, &mut init)
            .map_err(wrtc_err)?;
    }

    // 映像トランシーバー
    if sends_video(direction) {
        if let Some(source) = video_source {
            let video_track = factory
                .create_video_track(source, "video0")
                .map_err(wrtc_err)?;
            let media_track = video_track.cast_to_media_stream_track();
            let stream_ids = StringVector::new(0);
            let mut sender = pc.add_track(&media_track, &stream_ids).map_err(wrtc_err)?;
            // DegradationPreference を設定
            let mut params = sender.get_parameters();
            params.set_degradation_preference(Some(degradation_preference));
            sender.set_parameters(&params).map_err(wrtc_err)?;
            tracing::info!(target: "ayame", degradation = ?degradation_preference, "video track added with degradation preference");
        } else {
            let mut init = RtpTransceiverInit::new();
            init.set_direction(rtp_direction(direction));
            pc.add_transceiver(MediaType::Video, &mut init)
                .map_err(wrtc_err)?;
        }
    } else {
        let mut init = RtpTransceiverInit::new();
        init.set_direction(rtp_direction(direction));
        pc.add_transceiver(MediaType::Video, &mut init)
            .map_err(wrtc_err)?;
    }

    Ok(())
}

// ─── SDP 交換 ─────────────────────────────────────────────────────────────────

pub(super) fn create_and_send_offer(
    pc: &PeerConnection,
    cmd_tx: &tokio::sync::mpsc::Sender<SignalingCommand>,
) -> Result<(), BoxError> {
    let (sdp_tx, sdp_rx) = tokio::sync::oneshot::channel::<Result<String, String>>();
    let sdp_tx = Arc::new(Mutex::new(Some(sdp_tx)));

    struct CsdHandler {
        tx: SharedSender<Result<String, String>>,
    }
    impl CreateSessionDescriptionObserverHandler for CsdHandler {
        fn on_success(&mut self, desc: SessionDescription) {
            if let Ok(mut guard) = self.tx.lock()
                && let Some(t) = guard.take()
            {
                let _ = t.send(desc.to_string().map_err(|e| format!("{e:?}")));
            }
        }
        fn on_failure(&mut self, error: RtcError) {
            if let Ok(mut guard) = self.tx.lock()
                && let Some(t) = guard.take()
            {
                let _ = t.send(Err(error.message().unwrap_or_default()));
            }
        }
    }

    let mut observer =
        CreateSessionDescriptionObserver::new_with_handler(Box::new(CsdHandler { tx: sdp_tx }));

    {
        let mut options = PeerConnectionOfferAnswerOptions::new();
        pc.create_offer(&mut observer, &mut options);
    }

    let sdp = sdp_rx
        .blocking_recv()
        .map_err(|_| -> BoxError { "offer 作成チャンネルクローズ".into() })?
        .map_err(|e| -> BoxError { e.into() })?;

    let msg = SdpMessage {
        sdp_type: "offer",
        sdp: &sdp,
    };
    cmd_tx
        .blocking_send(SignalingCommand::SendText(nojson::Json(&msg).to_string()))
        .map_err(|_| -> BoxError { "シグナリングタスクが閉じています".into() })?;

    // SetLocalDescription
    let local_desc = SessionDescription::new(SdpType::Offer, &sdp).map_err(wrtc_err)?;
    let (sld_tx, sld_rx) = tokio::sync::oneshot::channel::<Result<(), String>>();
    let sld_tx = Arc::new(Mutex::new(Some(sld_tx)));

    struct SldHandler {
        tx: SharedSender<Result<(), String>>,
    }
    impl SetLocalDescriptionObserverHandler for SldHandler {
        fn on_set_local_description_complete(&mut self, error: RtcError) {
            if let Ok(mut guard) = self.tx.lock()
                && let Some(t) = guard.take()
            {
                let _ = t.send(if error.ok() {
                    Ok(())
                } else {
                    Err(error.message().unwrap_or_default())
                });
            }
        }
    }

    let sld_observer =
        SetLocalDescriptionObserver::new_with_handler(Box::new(SldHandler { tx: sld_tx }));
    pc.set_local_description(local_desc, &sld_observer);
    sld_rx
        .blocking_recv()
        .map_err(|_| -> BoxError { "SetLocalDescription チャンネルクローズ".into() })?
        .map_err(|e| -> BoxError { e.into() })?;

    info!(target: "sdp", "offer 送信");
    Ok(())
}

pub(super) fn handle_offer(
    pc: &PeerConnection,
    sdp: &str,
    cmd_tx: &tokio::sync::mpsc::Sender<SignalingCommand>,
    factory: &shiguredo_webrtc::PeerConnectionFactory,
    direction: &Direction,
    video_source: Option<VideoTrackSource>,
    degradation_preference: shiguredo_webrtc::DegradationPreference,
) -> Result<(), BoxError> {
    // SetRemoteDescription (offer)
    let desc = SessionDescription::new(SdpType::Offer, sdp).map_err(wrtc_err)?;
    let (srd_tx, srd_rx) = tokio::sync::oneshot::channel::<Result<(), String>>();
    let srd_tx = Arc::new(Mutex::new(Some(srd_tx)));

    struct SrdHandler {
        tx: SharedSender<Result<(), String>>,
    }
    impl SetRemoteDescriptionObserverHandler for SrdHandler {
        fn on_set_remote_description_complete(&mut self, error: RtcError) {
            if let Ok(mut guard) = self.tx.lock()
                && let Some(t) = guard.take()
            {
                let _ = t.send(if error.ok() {
                    Ok(())
                } else {
                    Err(error.message().unwrap_or_default())
                });
            }
        }
    }

    let srd_observer =
        SetRemoteDescriptionObserver::new_with_handler(Box::new(SrdHandler { tx: srd_tx }));
    pc.set_remote_description(desc, &srd_observer);
    srd_rx
        .blocking_recv()
        .map_err(|_| -> BoxError { "SetRemoteDescription チャンネルクローズ".into() })?
        .map_err(|e| -> BoxError { e.into() })?;

    // アンサー側: 映像送信がある場合は add_track で追加する
    if sends_video(direction)
        && let Some(ref source) = video_source
    {
        let video_track = factory
            .create_video_track(source, "video0")
            .map_err(wrtc_err)?;
        let media_track = video_track.cast_to_media_stream_track();
        let stream_ids = StringVector::new(0);
        let mut sender = pc.add_track(&media_track, &stream_ids).map_err(wrtc_err)?;
        // DegradationPreference を設定
        let mut params = sender.get_parameters();
        params.set_degradation_preference(Some(degradation_preference));
        sender.set_parameters(&params).map_err(wrtc_err)?;
        tracing::info!(target: "ayame", degradation = ?degradation_preference, "video track added with degradation preference (answer)");
    }

    // CreateAnswer
    let (sdp_tx, sdp_rx) = tokio::sync::oneshot::channel::<Result<String, String>>();
    let sdp_tx = Arc::new(Mutex::new(Some(sdp_tx)));

    struct CsdHandler2 {
        tx: SharedSender<Result<String, String>>,
    }
    impl CreateSessionDescriptionObserverHandler for CsdHandler2 {
        fn on_success(&mut self, desc: SessionDescription) {
            if let Ok(mut guard) = self.tx.lock()
                && let Some(t) = guard.take()
            {
                let _ = t.send(desc.to_string().map_err(|e| format!("{e:?}")));
            }
        }
        fn on_failure(&mut self, error: RtcError) {
            if let Ok(mut guard) = self.tx.lock()
                && let Some(t) = guard.take()
            {
                let _ = t.send(Err(error.message().unwrap_or_default()));
            }
        }
    }

    let mut observer =
        CreateSessionDescriptionObserver::new_with_handler(Box::new(CsdHandler2 { tx: sdp_tx }));

    {
        let mut options = PeerConnectionOfferAnswerOptions::new();
        pc.create_answer(&mut observer, &mut options);
    }

    let answer_sdp = sdp_rx
        .blocking_recv()
        .map_err(|_| -> BoxError { "answer 作成チャンネルクローズ".into() })?
        .map_err(|e| -> BoxError { e.into() })?;

    let msg = SdpMessage {
        sdp_type: "answer",
        sdp: &answer_sdp,
    };
    cmd_tx
        .blocking_send(SignalingCommand::SendText(nojson::Json(&msg).to_string()))
        .map_err(|_| -> BoxError { "シグナリングタスクが閉じています".into() })?;

    // SetLocalDescription (answer)
    let local_desc = SessionDescription::new(SdpType::Answer, &answer_sdp).map_err(wrtc_err)?;
    let (sld_tx, sld_rx) = tokio::sync::oneshot::channel::<Result<(), String>>();
    let sld_tx = Arc::new(Mutex::new(Some(sld_tx)));

    struct SldHandler2 {
        tx: SharedSender<Result<(), String>>,
    }
    impl SetLocalDescriptionObserverHandler for SldHandler2 {
        fn on_set_local_description_complete(&mut self, error: RtcError) {
            if let Ok(mut guard) = self.tx.lock()
                && let Some(t) = guard.take()
            {
                let _ = t.send(if error.ok() {
                    Ok(())
                } else {
                    Err(error.message().unwrap_or_default())
                });
            }
        }
    }

    let sld_observer =
        SetLocalDescriptionObserver::new_with_handler(Box::new(SldHandler2 { tx: sld_tx }));
    pc.set_local_description(local_desc, &sld_observer);
    sld_rx
        .blocking_recv()
        .map_err(|_| -> BoxError { "SetLocalDescription チャンネルクローズ".into() })?
        .map_err(|e| -> BoxError { e.into() })?;

    info!(target: "sdp", "answer 送信");
    Ok(())
}

pub(super) fn handle_answer(pc: &PeerConnection, sdp: &str) -> Result<(), BoxError> {
    let desc = SessionDescription::new(SdpType::Answer, sdp).map_err(wrtc_err)?;
    let (srd_tx, srd_rx) = tokio::sync::oneshot::channel::<Result<(), String>>();
    let srd_tx = Arc::new(Mutex::new(Some(srd_tx)));

    struct SrdHandler {
        tx: SharedSender<Result<(), String>>,
    }
    impl SetRemoteDescriptionObserverHandler for SrdHandler {
        fn on_set_remote_description_complete(&mut self, error: RtcError) {
            if let Ok(mut guard) = self.tx.lock()
                && let Some(t) = guard.take()
            {
                let _ = t.send(if error.ok() {
                    Ok(())
                } else {
                    Err(error.message().unwrap_or_default())
                });
            }
        }
    }

    let srd_observer =
        SetRemoteDescriptionObserver::new_with_handler(Box::new(SrdHandler { tx: srd_tx }));
    pc.set_remote_description(desc, &srd_observer);
    srd_rx
        .blocking_recv()
        .map_err(|_| -> BoxError { "SetRemoteDescription チャンネルクローズ".into() })?
        .map_err(|e| -> BoxError { e.into() })?;
    Ok(())
}
