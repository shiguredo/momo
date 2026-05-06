//! メトリクス HTTP サーバー
//!
//! `--metrics-port` で指定したポートで起動し、`GET /metrics` で JSON を返す。

use std::sync::Arc;

use shiguredo_http11::{RequestDecoder, Response};
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::net::{TcpListener, TcpStream};
use tokio::sync::{Mutex, mpsc, oneshot};
use tracing::{error, info};

use crate::error::BoxError;

/// stats を要求するチャンネルの送信側
///
/// `oneshot::Sender<String>` を送ると、PeerConnection 側が stats JSON を返す。
pub type StatsRequestTx = mpsc::Sender<oneshot::Sender<String>>;

/// メトリクスに必要な共有状態
pub struct MetricsState {
    version: String,
    libwebrtc: String,
    environment: String,
    /// 複数の PeerConnection からの stats 取得チャンネル
    providers: Mutex<Vec<StatsRequestTx>>,
}

impl MetricsState {
    pub fn new() -> Arc<Self> {
        let version = format!("WebRTC Native Client Momo {}", env!("CARGO_PKG_VERSION"));
        let libwebrtc = format!("webrtc-rs {}", shiguredo_webrtc::version());
        let environment = build_environment_string();
        Arc::new(Self {
            version,
            libwebrtc,
            environment,
            providers: Mutex::new(Vec::new()),
        })
    }

    /// stats プロバイダーを登録する
    pub async fn register(&self, tx: StatsRequestTx) {
        self.providers.lock().await.push(tx);
    }

    /// 全 PeerConnection から stats を収集して結合する
    ///
    /// 送信に失敗したプロバイダー（セッション終了済み）は自動的に除去する。
    async fn collect_stats(&self) -> String {
        let mut providers = self.providers.lock().await;
        let mut futures = Vec::new();
        let mut alive = Vec::new();

        for provider in providers.drain(..) {
            let (tx, rx) = oneshot::channel();
            if provider.send(tx).await.is_ok() {
                futures.push(rx);
                alive.push(provider);
            }
        }
        *providers = alive;
        drop(providers);

        let mut all_stats = Vec::new();
        for rx in futures {
            if let Ok(json) = rx.await
                && !json.is_empty()
                && json != "[]"
            {
                all_stats.push(json);
            }
        }

        // 各プロバイダーの JSON 配列を結合する
        let merged = if all_stats.is_empty() {
            "[]".to_string()
        } else {
            // 各要素は JSON 配列文字列。先頭の [ と末尾の ] を除去してカンマで結合する
            let inner: Vec<&str> = all_stats
                .iter()
                .filter_map(|s| {
                    let trimmed = s.trim();
                    if trimmed.len() > 2 {
                        Some(&trimmed[1..trimmed.len() - 1])
                    } else {
                        None
                    }
                })
                .collect();
            if inner.is_empty() {
                "[]".to_string()
            } else {
                format!("[{}]", inner.join(","))
            }
        };

        let body = nojson::object(|f| {
            f.member("version", self.version.as_str())?;
            f.member("libwebrtc", self.libwebrtc.as_str())?;
            f.member("environment", self.environment.as_str())?;
            f.member("stats", PreEncodedJson(&merged))
        });
        body.to_string()
    }
}

/// メトリクス HTTP サーバーを起動する
///
/// `allow_external_ip` が `true` の場合は `0.0.0.0` でバインドし、
/// `false` の場合は `127.0.0.1` でバインドする。
pub async fn run(
    port: u16,
    allow_external_ip: bool,
    state: Arc<MetricsState>,
) -> Result<(), BoxError> {
    let bind_addr = if allow_external_ip {
        "0.0.0.0"
    } else {
        "127.0.0.1"
    };
    let listener = TcpListener::bind((bind_addr, port)).await?;
    info!(target: "metrics", port = port, bind_addr = bind_addr, "metrics server started");

    loop {
        let (socket, _addr) = listener.accept().await?;
        let state = state.clone();
        tokio::spawn(async move {
            if let Err(e) = handle_request(socket, &state).await {
                error!(target: "metrics", error = %e, "metrics request error");
            }
        });
    }
}

/// HTTP リクエストを処理する
async fn handle_request(mut socket: TcpStream, state: &MetricsState) -> Result<(), BoxError> {
    let mut decoder = RequestDecoder::new();
    let mut buf = vec![0u8; 4096];

    let head = loop {
        let n = socket.read(&mut buf).await?;
        if n == 0 {
            return Ok(());
        }
        decoder.feed(&buf[..n])?;
        if let Some((head, _)) = decoder.decode_headers()? {
            break head;
        }
    };

    let path = head.uri.split('?').next().unwrap_or(&head.uri).to_owned();

    if path == "/metrics" {
        if head.method == "GET" {
            let body = state.collect_stats().await;
            let body_bytes = body.into_bytes();
            let resp = Response::new(200, "OK")
                .header("Content-Type", "application/json")
                .header("Content-Length", &body_bytes.len().to_string())
                .header("Access-Control-Allow-Origin", "*")
                .body(body_bytes);
            socket.write_all(&resp.encode()).await?;
        } else {
            let body = b"400 Bad Request";
            let resp = Response::new(400, "Bad Request")
                .header("Content-Type", "text/plain; charset=utf-8")
                .header("Content-Length", &body.len().to_string())
                .body(body.to_vec());
            socket.write_all(&resp.encode()).await?;
        }
    } else {
        let body = b"404 Not Found";
        let resp = Response::new(404, "Not Found")
            .header("Content-Type", "text/plain; charset=utf-8")
            .header("Content-Length", &body.len().to_string())
            .body(body.to_vec());
        socket.write_all(&resp.encode()).await?;
    }

    Ok(())
}

/// `"[ARCH] OS_DETAIL"` 形式の環境情報文字列を構築する
pub fn build_environment_string() -> String {
    let arch = std::env::consts::ARCH;
    let os_detail = get_os_detail();
    format!("[{}] {}", arch, os_detail)
}

/// OS の詳細情報を取得する
#[cfg(target_os = "macos")]
fn get_os_detail() -> String {
    // sw_vers コマンドから macOS バージョンを取得する
    let output = std::process::Command::new("sw_vers").output().ok();
    if let Some(output) = output {
        let stdout = String::from_utf8_lossy(&output.stdout);
        let mut name = None;
        let mut version = None;
        for line in stdout.lines() {
            if let Some(v) = line.strip_prefix("ProductName:") {
                name = Some(v.trim().to_string());
            } else if let Some(v) = line.strip_prefix("ProductVersion:") {
                version = Some(v.trim().to_string());
            }
        }
        if let (Some(name), Some(version)) = (name, version) {
            return format!("{} {}", name, version);
        }
    }
    "macOS".to_string()
}

/// OS の詳細情報を取得する
#[cfg(target_os = "linux")]
fn get_os_detail() -> String {
    // /etc/os-release から PRETTY_NAME を取得する
    if let Ok(content) = std::fs::read_to_string("/etc/os-release") {
        for line in content.lines() {
            if let Some(value) = line.strip_prefix("PRETTY_NAME=") {
                return value.trim_matches('"').to_string();
            }
        }
    }
    "Linux".to_string()
}

/// OS の詳細情報を取得する
#[cfg(not(any(target_os = "macos", target_os = "linux")))]
fn get_os_detail() -> String {
    std::env::consts::OS.to_string()
}

/// エンコード済み JSON 文字列をそのまま出力するラッパー
struct PreEncodedJson<'a>(&'a str);

impl nojson::DisplayJson for PreEncodedJson<'_> {
    fn fmt(&self, f: &mut nojson::JsonFormatter<'_, '_>) -> std::fmt::Result {
        std::fmt::Write::write_str(f.inner_mut(), self.0)
    }
}
