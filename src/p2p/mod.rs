//! P2P モードの実装
//!
//! TCP リスナー + HTTP 静的ファイルサーバー + WebSocket シグナリング + WebRTC

mod http;
mod message;
mod signaling;
mod webrtc;
mod websocket;

use std::path::PathBuf;
use std::sync::Arc;

use shiguredo_http11::{HttpHead, RequestDecoder};
use tokio::io::AsyncReadExt;
use tokio::net::{TcpListener, TcpStream};
use tracing::{error, info};

use crate::error::BoxError;
use crate::metrics::MetricsState;

/// P2P サーバーの設定
#[allow(dead_code)]
pub struct P2PConfig {
    pub port: u16,
    pub document_root: PathBuf,
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
    pub force_pixel_format: Option<shiguredo_video_device::PixelFormat>,
    pub degradation_preference: shiguredo_webrtc::DegradationPreference,
    pub video_codec_type: Option<String>,
    pub audio_codec_type: Option<String>,
    #[cfg(target_os = "linux")]
    pub serial: Option<crate::serial::SerialConfig>,
    pub metrics_state: Option<Arc<MetricsState>>,
}

/// P2P サーバーを起動する
pub async fn run(config: P2PConfig) -> Result<(), BoxError> {
    let config = Arc::new(config);
    let engine = Arc::new(webrtc::WebRtcEngine::new(&config)?);
    let listener = TcpListener::bind(("0.0.0.0", config.port)).await?;
    info!(target: "p2p", port = config.port, "server started");

    loop {
        let (socket, addr) = listener.accept().await?;
        let engine = engine.clone();
        let config = config.clone();
        tokio::spawn(async move {
            if let Err(e) = handle_connection(socket, engine, config).await {
                error!(target: "p2p", src_addr = %addr, error = %e, "connection error");
            }
        });
    }
}

/// HTTP リクエストを解析し、/ws なら WebSocket へ、それ以外は静的ファイルへ振り分ける
async fn handle_connection(
    mut socket: TcpStream,
    engine: Arc<webrtc::WebRtcEngine>,
    config: Arc<P2PConfig>,
) -> Result<(), BoxError> {
    let mut decoder = RequestDecoder::new();
    let mut raw_bytes: Vec<u8> = Vec::new();
    let mut buf = vec![0u8; 4096];

    let head = loop {
        let n = socket.read(&mut buf).await?;
        if n == 0 {
            return Ok(());
        }
        raw_bytes.extend_from_slice(&buf[..n]);
        decoder.feed(&buf[..n])?;
        if let Some((head, _)) = decoder.decode_headers()? {
            break head;
        }
    };

    let path = head
        .uri()
        .split('?')
        .next()
        .unwrap_or(head.uri())
        .to_owned();
    info!(target: "p2p", method = head.method(), %path, src_addr = %socket.peer_addr().map(|a| a.to_string()).unwrap_or_default(), "HTTP request");

    if head.method() == "GET" && path == "/ws" {
        let is_upgrade = head
            .get_header("Upgrade")
            .map(|v| v.eq_ignore_ascii_case("websocket"))
            .unwrap_or(false);
        if is_upgrade {
            return websocket::handle_websocket(
                socket,
                raw_bytes,
                engine,
                config.clone(),
                config.metrics_state.clone(),
            )
            .await;
        }
    }

    http::serve_static_file(socket, head.method(), &path, &config.document_root).await
}
