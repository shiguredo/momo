use std::sync::Arc;
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use rustls::pki_types::ServerName;
use rustls_pki_types::pem::PemObject;
use rustls_platform_verifier::ConfigVerifierExt;
use shiguredo_websocket::{
    ClientConnectionOptions, ConnectionEvent, ConnectionOutput, ConnectionState, TimerId,
    WebSocketClientConnection,
};
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::net::TcpStream;
use tokio::sync::mpsc;
use tokio::time::Instant;
use tokio_rustls::client::TlsStream;

use super::message::{ReceivedMessage, parse_received_message};
use crate::error::BoxError;

// ─── シグナリングコマンド / 通知 ──────────────────────────────────────────────

pub(super) enum SignalingCommand {
    SendText(String),
    Close,
}

pub(super) enum SignalingNotification {
    Connected,
    Message(ReceivedMessage),
    Error(BoxError),
    Closed,
}

// ─── RandomSource ─────────────────────────────────────────────────────────────

struct WebRtcRandomSource;

impl shiguredo_websocket::RandomSource for WebRtcRandomSource {
    fn masking_key(&mut self) -> [u8; 4] {
        let mut buf = [0u8; 4];
        aws_lc_rs::rand::fill(&mut buf).expect("BUG: 乱数生成に失敗しました");
        buf
    }

    fn nonce(&mut self) -> [u8; 16] {
        let mut buf = [0u8; 16];
        aws_lc_rs::rand::fill(&mut buf).expect("BUG: 乱数生成に失敗しました");
        buf
    }
}

// ─── ストリーム ───────────────────────────────────────────────────────────────

enum WsStream {
    Plain(TcpStream),
    Tls(Box<TlsStream<TcpStream>>),
}

impl WsStream {
    async fn read(&mut self, buf: &mut [u8]) -> std::io::Result<usize> {
        match self {
            WsStream::Plain(s) => s.read(buf).await,
            WsStream::Tls(s) => s.read(buf).await,
        }
    }

    async fn write_all(&mut self, buf: &[u8]) -> std::io::Result<()> {
        match self {
            WsStream::Plain(s) => s.write_all(buf).await,
            WsStream::Tls(s) => s.write_all(buf).await,
        }
    }

    async fn shutdown(&mut self) -> std::io::Result<()> {
        match self {
            WsStream::Plain(s) => s.shutdown().await,
            WsStream::Tls(s) => s.shutdown().await,
        }
    }
}

// ─── TLS ──────────────────────────────────────────────────────────────────────

fn create_tls_config(
    insecure: bool,
    client_cert: &Option<(String, String)>,
    ca_cert: &Option<String>,
) -> Result<Arc<rustls::ClientConfig>, BoxError> {
    let config = if insecure {
        // insecure モード: 証明書検証なし (ca_cert は無視)
        let builder = rustls::ClientConfig::builder()
            .dangerous()
            .with_custom_certificate_verifier(Arc::new(NoVerifier));
        if let Some((cert_pem, key_pem)) = client_cert {
            let certs = load_pem_certs(cert_pem)?;
            let key = load_pem_private_key(key_pem)?;
            builder
                .with_client_auth_cert(certs, key)
                .map_err(|e| -> BoxError { format!("client cert error: {e}").into() })?
        } else {
            builder.with_no_client_auth()
        }
    } else if let Some(ca_pem) = ca_cert {
        // カスタム CA 証明書モード: platform verifier を使わない
        let ca_certs = load_pem_certs(ca_pem)?;
        let mut root_store = rustls::RootCertStore::empty();
        for cert in ca_certs {
            root_store
                .add(cert)
                .map_err(|e| -> BoxError { format!("CA cert add error: {e}").into() })?;
        }
        let builder = rustls::ClientConfig::builder().with_root_certificates(root_store);
        if let Some((cert_pem, key_pem)) = client_cert {
            let certs = load_pem_certs(cert_pem)?;
            let key = load_pem_private_key(key_pem)?;
            builder
                .with_client_auth_cert(certs, key)
                .map_err(|e| -> BoxError { format!("client cert error: {e}").into() })?
        } else {
            builder.with_no_client_auth()
        }
    } else {
        // 通常モード: OS の CA ストアを使用
        let mut config = rustls::ClientConfig::with_platform_verifier()
            .map_err(|e| format!("TLS 設定エラー: {e}"))?;
        if let Some((cert_pem, key_pem)) = client_cert {
            let certs = load_pem_certs(cert_pem)?;
            let key = load_pem_private_key(key_pem)?;
            config.client_auth_cert_resolver = Arc::new(SingleCertResolver::new(certs, key)?);
        }
        config
    };
    Ok(Arc::new(config))
}

fn load_pem_certs(pem: &str) -> Result<Vec<rustls_pki_types::CertificateDer<'static>>, BoxError> {
    let certs: Vec<_> = rustls_pki_types::CertificateDer::pem_slice_iter(pem.as_bytes())
        .collect::<Result<_, _>>()
        .map_err(|e| -> BoxError { format!("PEM cert parse error: {e}").into() })?;
    if certs.is_empty() {
        return Err("no certificates found in PEM file".into());
    }
    Ok(certs)
}

fn load_pem_private_key(pem: &str) -> Result<rustls_pki_types::PrivateKeyDer<'static>, BoxError> {
    rustls_pki_types::PrivateKeyDer::from_pem_slice(pem.as_bytes())
        .map_err(|e| -> BoxError { format!("PEM key parse error: {e}").into() })
}

/// platform_verifier で構築済みの ClientConfig にクライアント証明書を設定するための Resolver
#[derive(Debug)]
struct SingleCertResolver {
    certified_key: Arc<rustls::sign::CertifiedKey>,
}

impl SingleCertResolver {
    fn new(
        certs: Vec<rustls::pki_types::CertificateDer<'static>>,
        key: rustls::pki_types::PrivateKeyDer<'static>,
    ) -> Result<Self, BoxError> {
        let signing_key = rustls::crypto::aws_lc_rs::default_provider()
            .key_provider
            .load_private_key(key)
            .map_err(|e| -> BoxError { format!("client key load error: {e}").into() })?;
        Ok(Self {
            certified_key: Arc::new(rustls::sign::CertifiedKey::new(certs, signing_key)),
        })
    }
}

impl rustls::client::ResolvesClientCert for SingleCertResolver {
    fn resolve(
        &self,
        _acceptable_issuers: &[&[u8]],
        _sigschemes: &[rustls::SignatureScheme],
    ) -> Option<Arc<rustls::sign::CertifiedKey>> {
        Some(self.certified_key.clone())
    }

    fn has_certs(&self) -> bool {
        true
    }
}

/// 証明書検証を行わないダミー検証器 (`--insecure` 用)
#[derive(Debug)]
struct NoVerifier;

impl rustls::client::danger::ServerCertVerifier for NoVerifier {
    fn verify_server_cert(
        &self,
        _end_entity: &rustls::pki_types::CertificateDer<'_>,
        _intermediates: &[rustls::pki_types::CertificateDer<'_>],
        _server_name: &rustls::pki_types::ServerName<'_>,
        _ocsp_response: &[u8],
        _now: rustls::pki_types::UnixTime,
    ) -> Result<rustls::client::danger::ServerCertVerified, rustls::Error> {
        Ok(rustls::client::danger::ServerCertVerified::assertion())
    }

    fn verify_tls12_signature(
        &self,
        _message: &[u8],
        _cert: &rustls::pki_types::CertificateDer<'_>,
        _dss: &rustls::DigitallySignedStruct,
    ) -> Result<rustls::client::danger::HandshakeSignatureValid, rustls::Error> {
        Ok(rustls::client::danger::HandshakeSignatureValid::assertion())
    }

    fn verify_tls13_signature(
        &self,
        _message: &[u8],
        _cert: &rustls::pki_types::CertificateDer<'_>,
        _dss: &rustls::DigitallySignedStruct,
    ) -> Result<rustls::client::danger::HandshakeSignatureValid, rustls::Error> {
        Ok(rustls::client::danger::HandshakeSignatureValid::assertion())
    }

    fn supported_verify_schemes(&self) -> Vec<rustls::SignatureScheme> {
        rustls::crypto::aws_lc_rs::default_provider()
            .signature_verification_algorithms
            .supported_schemes()
    }
}

async fn connect_tls(
    tcp: TcpStream,
    server_name: &str,
    config: Arc<rustls::ClientConfig>,
) -> Result<TlsStream<TcpStream>, BoxError> {
    let sn = ServerName::try_from(server_name.to_string())
        .map_err(|e| format!("サーバー名エラー: {e}"))?;
    let connector = tokio_rustls::TlsConnector::from(config);
    connector
        .connect(sn, tcp)
        .await
        .map_err(|e| format!("TLS 接続エラー: {e}").into())
}

// ─── シグナリングタスク ────────────────────────────────────────────────────────

fn parse_signaling_url(url: &str) -> Result<(String, u16, String, bool), BoxError> {
    let (scheme, rest) = url
        .split_once("://")
        .ok_or_else(|| -> BoxError { "URL にスキームがありません".into() })?;

    let use_tls = match scheme {
        "wss" | "https" => true,
        "ws" | "http" => false,
        _ => return Err(format!("未対応のスキーム: {scheme}").into()),
    };

    let (host_port, path) = match rest.find('/') {
        Some(idx) => (&rest[..idx], &rest[idx..]),
        None => (rest, "/"),
    };

    let (host, port) = match host_port.rsplit_once(':') {
        Some((h, p)) => {
            let port: u16 = p.parse().map_err(|_| format!("不正なポート: {p}"))?;
            (h.to_string(), port)
        }
        None => {
            let default_port = if use_tls { 443 } else { 80 };
            (host_port.to_string(), default_port)
        }
    };

    Ok((host, port, path.to_string(), use_tls))
}

/// WebSocket 接続パラメータ
struct WsConnectionParams {
    host: String,
    port: u16,
    path: String,
    use_tls: bool,
    insecure: bool,
    client_cert: Option<(String, String)>,
    ca_cert: Option<String>,
}

pub(super) fn start_signaling_task(
    signaling_url: &str,
    insecure: bool,
    client_cert: Option<(String, String)>,
    ca_cert: Option<String>,
) -> Result<
    (
        mpsc::Sender<SignalingCommand>,
        mpsc::Receiver<SignalingNotification>,
    ),
    BoxError,
> {
    let (host, port, path, use_tls) = parse_signaling_url(signaling_url)?;
    let params = WsConnectionParams {
        host,
        port,
        path,
        use_tls,
        insecure,
        client_cert,
        ca_cert,
    };
    let (cmd_tx, cmd_rx) = mpsc::channel::<SignalingCommand>(32);
    let (notify_tx, notify_rx) = mpsc::channel::<SignalingNotification>(64);

    tokio::spawn(async move {
        let result = run_signaling_loop(params, cmd_rx, &notify_tx).await;
        if let Err(e) = result {
            let _ = notify_tx.send(SignalingNotification::Error(e)).await;
        }
        let _ = notify_tx.send(SignalingNotification::Closed).await;
    });

    Ok((cmd_tx, notify_rx))
}

async fn run_signaling_loop(
    params: WsConnectionParams,
    mut cmd_rx: mpsc::Receiver<SignalingCommand>,
    notify_tx: &mpsc::Sender<SignalingNotification>,
) -> Result<(), BoxError> {
    let addr = format!("{}:{}", params.host, params.port);
    let tcp = TcpStream::connect(&addr).await?;

    let mut stream = if params.use_tls {
        let tls_config = create_tls_config(params.insecure, &params.client_cert, &params.ca_cert)?;
        WsStream::Tls(Box::new(connect_tls(tcp, &params.host, tls_config).await?))
    } else {
        WsStream::Plain(tcp)
    };

    let ws_options = ClientConnectionOptions::new(&params.host, &params.path);
    let mut ws = WebSocketClientConnection::new(ws_options, WebRtcRandomSource);
    ws.connect()
        .map_err(|e| format!("WebSocket 接続エラー: {e}"))?;

    let mut ping_dl: Option<Instant> = None;
    let mut pong_timeout_dl: Option<Instant> = None;
    let mut close_timeout_dl: Option<Instant> = None;
    let mut recv_buf = vec![0u8; 8192];

    // HTTP Upgrade リクエストを送出
    drain_output(
        &mut ws,
        &mut stream,
        &mut ping_dl,
        &mut pong_timeout_dl,
        &mut close_timeout_dl,
    )
    .await?;

    'main: loop {
        let earliest = earliest_deadline(&[ping_dl, pong_timeout_dl, close_timeout_dl]);

        tokio::select! {
            // ソケット受信
            result = stream.read(&mut recv_buf) => {
                let n = result?;
                if n == 0 {
                    break 'main;
                }

                let now_millis = SystemTime::now()
                    .duration_since(UNIX_EPOCH)
                    .unwrap_or_default()
                    .as_millis() as u64;
                let now = shiguredo_websocket::Timestamp::from_millis(now_millis);

                if let Err(e) = ws.feed_recv_buf(&recv_buf[..n], now) {
                    let _ = notify_tx
                        .send(SignalingNotification::Error(
                            format!("WebSocket 受信エラー: {e}").into(),
                        ))
                        .await;
                    break 'main;
                }

                while let Some(event) = ws.poll_event() {
                    match event {
                        ConnectionEvent::Connected { .. } => {
                            let _ = notify_tx.send(SignalingNotification::Connected).await;
                        }
                        ConnectionEvent::TextMessage(text) => {
                            match parse_received_message(&text) {
                                Ok(msg) => {
                                    let _ = notify_tx
                                        .send(SignalingNotification::Message(msg))
                                        .await;
                                }
                                Err(e) => {
                                    let _ =
                                        notify_tx.send(SignalingNotification::Error(e)).await;
                                }
                            }
                        }
                        ConnectionEvent::Close { .. } => {
                            let _ = ws.close(shiguredo_websocket::CloseCode::NORMAL, "");
                        }
                        ConnectionEvent::StateChanged(ConnectionState::Closed) => {
                            break 'main;
                        }
                        ConnectionEvent::Error(msg) => {
                            let _ = notify_tx
                                .send(SignalingNotification::Error(
                                    format!("WebSocket エラー: {msg}").into(),
                                ))
                                .await;
                            break 'main;
                        }
                        _ => {}
                    }
                }

                drain_output(
                    &mut ws,
                    &mut stream,
                    &mut ping_dl,
                    &mut pong_timeout_dl,
                    &mut close_timeout_dl,
                )
                .await?;
            }

            // コマンド受信
            cmd = cmd_rx.recv() => {
                match cmd {
                    Some(SignalingCommand::SendText(text)) => {
                        ws.send_text(&text)
                            .map_err(|e| format!("WebSocket 送信エラー: {e}"))?;
                        drain_output(
                            &mut ws,
                            &mut stream,
                            &mut ping_dl,
                            &mut pong_timeout_dl,
                            &mut close_timeout_dl,
                        )
                        .await?;
                    }
                    Some(SignalingCommand::Close) => {
                        let _ = ws
                            .close(shiguredo_websocket::CloseCode::NORMAL, "client close");
                        drain_output(
                            &mut ws,
                            &mut stream,
                            &mut ping_dl,
                            &mut pong_timeout_dl,
                            &mut close_timeout_dl,
                        )
                        .await?;
                    }
                    None => break 'main,
                }
            }

            // タイマー発火
            _ = tokio::time::sleep_until(earliest) => {
                let now = Instant::now();
                for (dl, id) in [
                    (&mut ping_dl, TimerId::Ping),
                    (&mut pong_timeout_dl, TimerId::PongTimeout),
                    (&mut close_timeout_dl, TimerId::CloseTimeout),
                ] {
                    if dl.map(|d| now >= d).unwrap_or(false) {
                        *dl = None;
                        ws.handle_timer(id)
                            .map_err(|e| format!("タイマー処理エラー: {e}"))?;
                    }
                }
                drain_output(
                    &mut ws,
                    &mut stream,
                    &mut ping_dl,
                    &mut pong_timeout_dl,
                    &mut close_timeout_dl,
                )
                .await?;
            }
        }
    }

    Ok(())
}

async fn drain_output(
    ws: &mut WebSocketClientConnection<WebRtcRandomSource>,
    stream: &mut WsStream,
    ping_dl: &mut Option<Instant>,
    pong_timeout_dl: &mut Option<Instant>,
    close_timeout_dl: &mut Option<Instant>,
) -> Result<(), BoxError> {
    while let Some(output) = ws.poll_output() {
        match output {
            ConnectionOutput::SendData(data) => {
                stream.write_all(&data).await?;
            }
            ConnectionOutput::SetTimer {
                id,
                duration_millis,
            } => {
                let dl = Instant::now() + Duration::from_millis(duration_millis);
                match id {
                    TimerId::Ping => *ping_dl = Some(dl),
                    TimerId::PongTimeout => *pong_timeout_dl = Some(dl),
                    TimerId::CloseTimeout => *close_timeout_dl = Some(dl),
                }
            }
            ConnectionOutput::ClearTimer { id } => match id {
                TimerId::Ping => *ping_dl = None,
                TimerId::PongTimeout => *pong_timeout_dl = None,
                TimerId::CloseTimeout => *close_timeout_dl = None,
            },
            ConnectionOutput::CloseConnection => {
                let _ = stream.shutdown().await;
            }
        }
    }
    Ok(())
}

fn earliest_deadline(deadlines: &[Option<Instant>]) -> Instant {
    deadlines
        .iter()
        .flatten()
        .copied()
        .min()
        .unwrap_or_else(|| Instant::now() + Duration::from_secs(3600))
}
