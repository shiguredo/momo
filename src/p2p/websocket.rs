use std::sync::Arc;
use std::time::Duration;

use shiguredo_websocket::{
    CloseCode, ConnectionEvent, ConnectionOutput, ConnectionState, ServerConnectionOptions,
    TimerId, WebSocketServerConnection,
};
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::net::TcpStream;
use tokio::sync::mpsc;
use tokio::time::Instant;
use tracing::{debug, info, warn};

use super::P2PConfig;
use super::signaling::handle_signaling;
use super::webrtc::{Peer, WebRtcEngine};
use crate::error::BoxError;
use crate::metrics::MetricsState;

/// WebSocket セッション：ハンドシェイク → シグナリングループ
pub(super) async fn handle_websocket(
    mut socket: TcpStream,
    initial_bytes: Vec<u8>,
    engine: Arc<WebRtcEngine>,
    config: Arc<P2PConfig>,
    metrics_state: Option<Arc<MetricsState>>,
) -> Result<(), BoxError> {
    let src_addr = socket.peer_addr()?;
    let mut ws = WebSocketServerConnection::new(ServerConnectionOptions::new());

    // HTTP アップグレードリクエストを WebSocket ステートマシンに供給
    ws.feed_recv_buf(&initial_bytes)?;

    // ハンドシェイクを受諾 → 101 Switching Protocols を出力キューに積む
    ws.accept_handshake_auto()?;
    info!(target: "ws", src_addr = %src_addr, path = "/ws", "opened");

    // タイマー期限（None = タイマー未設定）
    let mut ping_dl: Option<Instant> = None;
    let mut pong_timeout_dl: Option<Instant> = None;
    let mut close_timeout_dl: Option<Instant> = None;

    // 初期出力フラッシュ（101 Switching Protocols を含む）
    drain_ws_output(
        &mut ws,
        &mut socket,
        &mut ping_dl,
        &mut pong_timeout_dl,
        &mut close_timeout_dl,
    )
    .await?;

    // Connected イベントを確認
    while let Some(event) = ws.poll_event() {
        if matches!(
            event,
            ConnectionEvent::StateChanged(ConnectionState::Closed)
        ) {
            info!(target: "ws", src_addr = %src_addr, "closed");
            return Ok(());
        }
    }

    // WebRTC → WebSocket のシグナリングチャンネル
    let (sig_tx, mut sig_rx) = mpsc::unbounded_channel::<String>();
    let mut peer: Option<Peer> = None;
    let mut read_buf = vec![0u8; 8192];

    // メトリクス stats チャンネル
    let (stats_tx, mut stats_rx) = mpsc::channel::<tokio::sync::oneshot::Sender<String>>(1);
    if let Some(ref ms) = metrics_state {
        ms.register(stats_tx).await;
    }

    'main: loop {
        let earliest = earliest_deadline(&[ping_dl, pong_timeout_dl, close_timeout_dl]);

        tokio::select! {
            // ソケット受信
            result = socket.read(&mut read_buf) => {
                let n = result?;
                if n == 0 {
                    break 'main;
                }
                ws.feed_recv_buf(&read_buf[..n])?;

                while let Some(event) = ws.poll_event() {
                    match event {
                        ConnectionEvent::TextMessage(text) => {
                            debug!(target: "sig", msg = &text[..text.len().min(120)], "recv");
                            if let Err(e) = handle_signaling(
                                &text,
                                &mut peer,
                                &engine,
                                &sig_tx,
                                &config,
                            ).await {
                                warn!(target: "sig", error = %e, "signaling error");
                            }
                        }
                        ConnectionEvent::Close { .. } => {
                            // state が Closed/Disconnected の場合はエラーを無視する
                            let _ = ws.close(CloseCode::NORMAL, "");
                        }
                        ConnectionEvent::StateChanged(state) => {
                            info!(target: "ws", src_addr = %src_addr, state = ?state, "state changed");
                            if matches!(state, ConnectionState::Closed) {
                                info!(target: "ws", src_addr = %src_addr, "closed");
                                break 'main;
                            }
                        }
                        _ => {}
                    }
                }

                drain_ws_output(
                    &mut ws,
                    &mut socket,
                    &mut ping_dl,
                    &mut pong_timeout_dl,
                    &mut close_timeout_dl,
                ).await?;
            }

            // WebRTC シグナリングメッセージ送信
            msg = sig_rx.recv() => {
                if let Some(msg) = msg {
                    debug!(target: "sig", msg = &msg[..msg.len().min(120)], "send");
                    if ws.send_text(&msg).is_err() {
                        break 'main;
                    }
                    drain_ws_output(
                        &mut ws,
                        &mut socket,
                        &mut ping_dl,
                        &mut pong_timeout_dl,
                        &mut close_timeout_dl,
                    ).await?;
                }
            }

            // メトリクス stats リクエスト
            Some(reply_tx) = stats_rx.recv() => {
                if let Some(ref peer) = peer {
                    peer.pc.get_stats(move |report| {
                        let json = report.to_json().unwrap_or_else(|_| "[]".to_string());
                        let _ = reply_tx.send(json);
                    });
                } else {
                    let _ = reply_tx.send("[]".to_string());
                }
            }

            // タイマー発火
            _ = tokio::time::sleep_until(earliest) => {
                let now_inst = Instant::now();

                for (dl, id) in [
                    (&mut ping_dl, TimerId::Ping),
                    (&mut pong_timeout_dl, TimerId::PongTimeout),
                    (&mut close_timeout_dl, TimerId::CloseTimeout),
                ] {
                    if dl.map(|d| now_inst >= d).unwrap_or(false) {
                        *dl = None;
                        ws.handle_timer(id)?;
                    }
                }

                drain_ws_output(
                    &mut ws,
                    &mut socket,
                    &mut ping_dl,
                    &mut pong_timeout_dl,
                    &mut close_timeout_dl,
                ).await?;
            }
        }
    }

    Ok(())
}

/// WebSocket の出力キューをすべてソケットに書き出し、タイマーを更新する
async fn drain_ws_output(
    ws: &mut WebSocketServerConnection,
    socket: &mut TcpStream,
    ping_dl: &mut Option<Instant>,
    pong_timeout_dl: &mut Option<Instant>,
    close_timeout_dl: &mut Option<Instant>,
) -> Result<(), BoxError> {
    while let Some(output) = ws.poll_output() {
        match output {
            ConnectionOutput::SendData(data) => {
                socket.write_all(&data).await?;
            }
            ConnectionOutput::SetTimer {
                id,
                duration_millis,
            } => {
                let dl = Instant::now() + Duration::from_millis(duration_millis);
                timer_set(id, dl, ping_dl, pong_timeout_dl, close_timeout_dl);
            }
            ConnectionOutput::ClearTimer { id } => {
                timer_clear(id, ping_dl, pong_timeout_dl, close_timeout_dl);
            }
            ConnectionOutput::CloseConnection => {
                let _ = socket.shutdown().await;
            }
        }
    }
    Ok(())
}

fn timer_set(
    id: TimerId,
    dl: Instant,
    ping: &mut Option<Instant>,
    pong: &mut Option<Instant>,
    close: &mut Option<Instant>,
) {
    match id {
        TimerId::Ping => *ping = Some(dl),
        TimerId::PongTimeout => *pong = Some(dl),
        TimerId::CloseTimeout => *close = Some(dl),
    }
}

fn timer_clear(
    id: TimerId,
    ping: &mut Option<Instant>,
    pong: &mut Option<Instant>,
    close: &mut Option<Instant>,
) {
    match id {
        TimerId::Ping => *ping = None,
        TimerId::PongTimeout => *pong = None,
        TimerId::CloseTimeout => *close = None,
    }
}

fn earliest_deadline(deadlines: &[Option<Instant>]) -> Instant {
    deadlines
        .iter()
        .flatten()
        .copied()
        .min()
        .unwrap_or_else(|| Instant::now() + Duration::from_secs(3600))
}
