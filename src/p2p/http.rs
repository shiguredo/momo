use std::path::Path;

use shiguredo_http11::Response;
use tokio::io::AsyncWriteExt;
use tokio::net::TcpStream;

use crate::error::BoxError;

/// 静的ファイルをレスポンスとして送信する
pub(super) async fn serve_static_file(
    mut socket: TcpStream,
    method: &str,
    request_path: &str,
    doc_root: &Path,
) -> Result<(), BoxError> {
    if request_path.contains("..") {
        let body = b"403 Forbidden";
        let resp = Response::new(403, "Forbidden")
            .header("Content-Type", "text/plain; charset=utf-8")
            .header("Content-Length", &body.len().to_string())
            .body(body.to_vec());
        socket.write_all(&resp.encode()).await?;
        return Ok(());
    }

    let rel = request_path.trim_start_matches('/');
    let mut file_path = doc_root.join(rel);
    if file_path.is_dir() {
        file_path.push("index.html");
    }

    match tokio::fs::read(&file_path).await {
        Ok(body) => {
            let ct = mime_type(&file_path);
            let resp = if method == "HEAD" {
                Response::new(200, "OK")
                    .header("Content-Type", ct)
                    .header("Content-Length", &body.len().to_string())
            } else {
                Response::new(200, "OK")
                    .header("Content-Type", ct)
                    .header("Content-Length", &body.len().to_string())
                    .body(body)
            };
            socket.write_all(&resp.encode()).await?;
        }
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => {
            let body = b"404 Not Found";
            let resp = Response::new(404, "Not Found")
                .header("Content-Type", "text/plain; charset=utf-8")
                .header("Content-Length", &body.len().to_string())
                .body(body.to_vec());
            socket.write_all(&resp.encode()).await?;
        }
        Err(e) => {
            let body = format!("500 Internal Server Error: {e}").into_bytes();
            let resp = Response::new(500, "Internal Server Error")
                .header("Content-Type", "text/plain; charset=utf-8")
                .header("Content-Length", &body.len().to_string())
                .body(body);
            socket.write_all(&resp.encode()).await?;
        }
    }

    Ok(())
}

fn mime_type(path: &Path) -> &'static str {
    match path.extension().and_then(|e| e.to_str()) {
        Some("html") | Some("htm") => "text/html; charset=utf-8",
        Some("js") | Some("mjs") => "application/javascript",
        Some("css") => "text/css",
        Some("json") => "application/json",
        Some("png") => "image/png",
        Some("jpg") | Some("jpeg") => "image/jpeg",
        Some("gif") => "image/gif",
        Some("svg") => "image/svg+xml",
        Some("ico") => "image/x-icon",
        Some("wasm") => "application/wasm",
        _ => "application/octet-stream",
    }
}
