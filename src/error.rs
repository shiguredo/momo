/// ボックス化されたエラー型
pub type BoxError = Box<dyn std::error::Error + Send + Sync>;

/// `shiguredo_webrtc::Error` は `Sync` でないため `BoxError` に直接変換できない
pub fn wrtc_err(e: shiguredo_webrtc::Error) -> BoxError {
    e.to_string().into()
}
