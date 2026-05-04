use std::sync::Arc;

use nojson::RawJson;
use shiguredo_webrtc::{IceCandidate, SdpType, SessionDescription};
use tokio::sync::mpsc;
use tracing::{debug, info, warn};

use super::P2PConfig;
use super::message::json_accept;
use super::webrtc::{Peer, WebRtcEngine, create_peer, process_offer, set_remote_desc};
use crate::error::{BoxError, wrtc_err};

// ─── シグナリング ──────────────────────────────────────────────────────────

/// WebSocket テキストメッセージを JSON として解析してシグナリング処理する
pub(super) async fn handle_signaling(
    text: &str,
    peer: &mut Option<Peer>,
    engine: &Arc<WebRtcEngine>,
    sig_tx: &mpsc::UnboundedSender<String>,
    #[cfg_attr(not(target_os = "linux"), allow(unused_variables))] config: &Arc<P2PConfig>,
) -> Result<(), BoxError> {
    let raw = RawJson::parse(text).map_err(|e| format!("JSON パースエラー: {e}"))?;

    let msg_type: String = raw.value().to_member("type")?.required()?.try_into()?;

    match msg_type.as_str() {
        "register" => {
            let is_exist = peer.is_some();
            let _ = sig_tx.send(json_accept(is_exist));
            info!(target: "sig", is_exist_user = is_exist, "register -> accept");
        }

        "offer" => {
            let sdp: String = raw.value().to_member("sdp")?.required()?.try_into()?;

            info!(target: "sdp", "offer received");
            *peer = None;
            #[cfg(target_os = "linux")]
            let new_peer = create_peer(
                engine,
                sig_tx.clone(),
                config.no_google_stun,
                config.serial.clone(),
            )?;
            #[cfg(not(target_os = "linux"))]
            let new_peer = create_peer(engine, sig_tx.clone(), config.no_google_stun)?;
            process_offer(&new_peer.pc, sdp, sig_tx).await?;
            *peer = Some(new_peer);
        }

        "answer" => {
            if let Some(p) = peer {
                let sdp: String = raw.value().to_member("sdp")?.required()?.try_into()?;

                info!(target: "sdp", "answer received");
                let desc = SessionDescription::new(SdpType::Answer, &sdp).map_err(wrtc_err)?;
                set_remote_desc(&p.pc, desc).await?;
            } else {
                warn!(target: "sdp", "answer received but no PeerConnection");
            }
        }

        "candidate" => {
            if let Some(p) = peer {
                let ice = raw.value().to_member("ice")?.required()?;
                let candidate: String = ice.to_member("candidate")?.required()?.try_into()?;
                let sdp_mid: String = ice.to_member("sdpMid")?.required()?.try_into()?;
                let sdp_mline_index: i32 =
                    ice.to_member("sdpMLineIndex")?.required()?.try_into()?;

                debug!(target: "sdp", %candidate, "candidate received");
                let ice_candidate =
                    IceCandidate::new(&sdp_mid, sdp_mline_index, &candidate).map_err(wrtc_err)?;
                p.pc.add_ice_candidate(&ice_candidate).map_err(wrtc_err)?;
            } else {
                warn!(target: "sdp", "candidate received but no PeerConnection");
            }
        }

        "close" | "bye" => {
            info!(target: "sig", "close/bye -> closing PeerConnection");
            *peer = None;
        }

        other => {
            warn!(target: "sig", msg_type = other, "unknown message type");
        }
    }

    Ok(())
}
