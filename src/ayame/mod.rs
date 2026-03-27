//! Ayame モードの実装
//!
//! ayame-rust-sdk の接続ロジックを 1 ファイルに統合した実装。
//! 映像は raden でアニメーションフレームを生成するフェイクソースを使用する。

mod message;
mod signaling;
mod webrtc;

use std::sync::Arc;

use nojson::Json;
use shiguredo_webrtc::{IceCandidate, VideoTrackSource};
use tokio::sync::mpsc;
use tracing::{info, warn};

use crate::metrics::MetricsState;

use self::message::{PongMessage, ReceivedMessage, RegisterMessage};
use self::signaling::{SignalingCommand, SignalingNotification, start_signaling_task};
use self::webrtc::{
    AyameEngine, add_transceivers, create_and_send_offer, create_peer, handle_answer, handle_offer,
};
use crate::error::BoxError;

// ─── 設定 ─────────────────────────────────────────────────────────────────────

#[allow(dead_code)]
pub struct AyameConfig {
    pub signaling_url: String,
    pub room_id: String,
    pub client_id: Option<String>,
    pub signaling_key: Option<String>,
    pub direction: Direction,
    pub no_google_stun: bool,
    pub no_audio_device: bool,
    pub no_video_input_device: bool,
    pub fake_capture_device: bool,
    pub use_libcamera: bool,
    pub use_libcamera_native: bool,
    pub libcamera_controls: Vec<(String, String)>,
    pub use_v4l2_encoder: bool,
    pub openh264_lib: Option<shiguredo_openh264::Openh264Library>,
    pub video_input_device: Option<String>,
    pub audio_input_device: Option<String>,
    pub video_width: i32,
    pub video_height: i32,
    pub framerate: u32,
    pub insecure: bool,
    pub force_pixel_format: Option<shiguredo_video_device::PixelFormat>,
    /// クライアント証明書 (cert_pem, key_pem)
    pub client_cert: Option<(String, String)>,
    /// CA 証明書 (PEM)
    pub ca_cert: Option<String>,
    pub degradation_preference: shiguredo_webrtc::DegradationPreference,
    pub video_codec_type: Option<String>,
    pub audio_codec_type: Option<String>,
    #[cfg(target_os = "linux")]
    pub serial: Option<crate::serial::SerialConfig>,
}

pub enum Direction {
    SendRecv,
    SendOnly,
    RecvOnly,
}

// ─── 公開 API ─────────────────────────────────────────────────────────────────

/// Ayame サーバーに接続する。
pub async fn run(
    config: AyameConfig,
    metrics_state: Option<Arc<MetricsState>>,
) -> Result<(), BoxError> {
    // クライアント ID (未指定時はランダム 8 バイトを hex 文字列に)
    let client_id = config.client_id.clone().unwrap_or_else(|| {
        let mut bytes = [0u8; 8];
        aws_lc_rs::rand::fill(&mut bytes).expect("BUG: 乱数生成に失敗しました");
        bytes.iter().map(|b| format!("{b:02x}")).collect()
    });

    // WebRTC エンジン初期化
    let mut engine = AyameEngine::new(&config)?;

    // シグナリングタスク起動
    let (cmd_tx, mut notify_rx) = start_signaling_task(
        &config.signaling_url,
        config.insecure,
        config.client_cert,
        config.ca_cert,
    )?;

    // WebSocket 接続完了を待つ
    match notify_rx.recv().await {
        Some(SignalingNotification::Connected) => {}
        Some(SignalingNotification::Error(e)) => return Err(e),
        Some(SignalingNotification::Closed) => {
            return Err("ハンドシェイク前に接続が閉じました".into());
        }
        _ => return Err("予期しない通知を受信しました".into()),
    }
    info!(target: "ayame", "WebSocket 接続完了");

    // register 送信
    let register = RegisterMessage {
        room_id: &config.room_id,
        client_id: &client_id,
        signaling_key: config.signaling_key.as_deref(),
    };
    cmd_tx
        .send(SignalingCommand::SendText(Json(&register).to_string()))
        .await
        .map_err(|_| -> BoxError { "シグナリングタスクが閉じています".into() })?;
    info!(target: "ayame", room_id = %config.room_id, client_id = %client_id, "register 送信");

    // accept/reject 待ち
    let accept = loop {
        match notify_rx.recv().await {
            Some(SignalingNotification::Message(ReceivedMessage::Accept(accept))) => {
                break accept;
            }
            Some(SignalingNotification::Message(ReceivedMessage::Reject { reason })) => {
                return Err(format!("reject: {reason}").into());
            }
            Some(SignalingNotification::Error(e)) => return Err(e),
            Some(SignalingNotification::Closed) | None => {
                return Err("accept 前に接続が閉じました".into());
            }
            _ => continue,
        }
    };
    info!(target: "ayame", is_exist_user = accept.is_exist_user, "accept 受信");

    // PeerConnection 作成
    let mut peer = create_peer(
        &engine,
        cmd_tx.clone(),
        &accept,
        config.no_google_stun,
        #[cfg(target_os = "linux")]
        config.serial.clone(),
    )?;

    // メトリクス stats チャンネル
    let (stats_tx, mut stats_rx) = mpsc::channel::<tokio::sync::oneshot::Sender<String>>(1);
    if let Some(ref ms) = metrics_state {
        ms.register(stats_tx).await;
    }

    // オファー側 / アンサー側 で分岐
    // WebRTC の SDP 操作は C++ スレッドからのコールバックを同期的に待つため block_in_place を使用
    let mut pending_video_source: Option<VideoTrackSource> = None;
    if accept.is_exist_user {
        tokio::task::block_in_place(|| {
            add_transceivers(
                &peer.pc,
                &engine.factory,
                &config.direction,
                engine.video_track_source.as_ref(),
                config.degradation_preference,
                config.video_codec_type.as_deref(),
                config.audio_codec_type.as_deref(),
            )?;
            create_and_send_offer(&peer.pc, &cmd_tx)
        })?;
    } else {
        pending_video_source = engine.video_track_source.take();
    }

    // メッセージループ
    loop {
        tokio::select! {
            notification = notify_rx.recv() => {
                match notification {
                    Some(SignalingNotification::Message(msg)) => match msg {
                        ReceivedMessage::Offer { sdp } => {
                            let vs = pending_video_source.take();
                            tokio::task::block_in_place(|| {
                                handle_offer(
                                    &peer.pc,
                                    &sdp,
                                    &cmd_tx,
                                    &engine.factory,
                                    &config.direction,
                                    vs,
                                    config.degradation_preference,
                                    config.video_codec_type.as_deref(),
                                    config.audio_codec_type.as_deref(),
                                )
                            })?;
                        }
                        ReceivedMessage::Answer { sdp } => {
                            tokio::task::block_in_place(|| handle_answer(&peer.pc, &sdp))?;
                        }
                        ReceivedMessage::Candidate {
                            candidate,
                            sdp_mid,
                            sdp_mline_index,
                        } => {
                            if let Ok(ice) = IceCandidate::new(&sdp_mid, sdp_mline_index, &candidate) {
                                let _ = peer.pc.add_ice_candidate(&ice);
                            }
                        }
                        ReceivedMessage::Ping => {
                            let _ = cmd_tx
                                .send(SignalingCommand::SendText(Json(&PongMessage).to_string()))
                                .await;
                            info!(target: "ayame", "ping → pong");
                        }
                        ReceivedMessage::Bye => {
                            info!(target: "ayame", "bye 受信、切断します");
                            break;
                        }
                        _ => {}
                    },
                    Some(SignalingNotification::Error(e)) => {
                        warn!(target: "ayame", error = %e, "シグナリングエラー");
                        break;
                    }
                    Some(SignalingNotification::Closed) | None => {
                        info!(target: "ayame", "シグナリング接続がクローズされました");
                        break;
                    }
                    Some(SignalingNotification::Connected) => {}
                }
            }

            Some(reply_tx) = stats_rx.recv() => {
                peer.pc.get_stats(move |report| {
                    let json = report.to_json().unwrap_or_else(|_| "[]".to_string());
                    let _ = reply_tx.send(json);
                });
            }
        }
    }

    let _ = cmd_tx.send(SignalingCommand::Close).await;
    Ok(())
}
