//! プレビューウィンドウ (raw_player) の共通モジュール
//!
//! キャプチャ映像を SDL3 ウィンドウでリアルタイム表示する。

use std::sync::mpsc::{Receiver, SyncSender};
use std::time::Duration;

use raw_player::VideoPlayer;
use shiguredo_webrtc::I420Buffer;

use crate::error::BoxError;

// ─── プレビューフレーム ──────────────────────────────────────────────────────

/// I420 プレーンのコピーを保持するフレームデータ
pub struct PreviewFrame {
    pub y: Vec<u8>,
    pub u: Vec<u8>,
    pub v: Vec<u8>,
    pub width: i32,
    pub height: i32,
    pub pts_us: i64,
}

// ─── チャネル生成 ────────────────────────────────────────────────────────────

/// bounded チャネル (容量 2) を生成する
///
/// キュー満杯時は try_send でドロップする想定。プレビューはベストエフォート。
pub fn create_preview_channel() -> (SyncSender<PreviewFrame>, Receiver<PreviewFrame>) {
    std::sync::mpsc::sync_channel(2)
}

// ─── I420Buffer からのフレーム抽出 ───────────────────────────────────────────

/// I420Buffer から Y/U/V プレーンを stride 考慮でコピーして PreviewFrame を生成する
pub fn extract_preview_frame(
    buf: &I420Buffer,
    width: i32,
    height: i32,
    pts_us: i64,
) -> PreviewFrame {
    use shiguredo_webrtc::ffi::*;

    let w = width as usize;
    let h = height as usize;
    let chroma_w = w / 2;
    let chroma_h = h / 2;

    let mut y = vec![0u8; w * h];
    let mut u = vec![0u8; chroma_w * chroma_h];
    let mut v = vec![0u8; chroma_w * chroma_h];

    // SAFETY: I420Buffer が生きている間、派生したポインタは有効である。
    // MutableData* は書き込み用だが読み取りにも使える。
    unsafe {
        let refcounted = buf.as_refcounted_ptr();
        let raw = webrtc_I420Buffer_refcounted_get(refcounted);

        let y_ptr = webrtc_I420Buffer_MutableDataY(raw);
        let u_ptr = webrtc_I420Buffer_MutableDataU(raw);
        let v_ptr = webrtc_I420Buffer_MutableDataV(raw);
        let stride_y = webrtc_I420Buffer_StrideY(raw) as usize;
        let stride_u = webrtc_I420Buffer_StrideU(raw) as usize;
        let stride_v = webrtc_I420Buffer_StrideV(raw) as usize;

        // Y プレーン: 行ごとに width バイトコピー
        for row in 0..h {
            let src = std::slice::from_raw_parts(y_ptr.add(row * stride_y), w);
            y[row * w..row * w + w].copy_from_slice(src);
        }

        // U プレーン
        for row in 0..chroma_h {
            let src = std::slice::from_raw_parts(u_ptr.add(row * stride_u), chroma_w);
            u[row * chroma_w..row * chroma_w + chroma_w].copy_from_slice(src);
        }

        // V プレーン
        for row in 0..chroma_h {
            let src = std::slice::from_raw_parts(v_ptr.add(row * stride_v), chroma_w);
            v[row * chroma_w..row * chroma_w + chroma_w].copy_from_slice(src);
        }
    }

    PreviewFrame {
        y,
        u,
        v,
        width,
        height,
        pts_us,
    }
}

// ─── プレビューイベントループ ────────────────────────────────────────────────

/// メインスレッドで SDL3 イベントループを実行する
///
/// - `frame_rx` からフレームを受信して VideoPlayer に enqueue する
/// - `poll_events()` でレンダリング + イベント処理
/// - `shutdown_rx` で Sora 接続終了を検知して終了
pub fn run_preview_loop(
    window_width: i32,
    window_height: i32,
    frame_rx: Receiver<PreviewFrame>,
    shutdown_rx: Receiver<()>,
) -> Result<(), BoxError> {
    raw_player::init().map_err(|e| -> BoxError { format!("SDL3 init failed: {e}").into() })?;

    let player = VideoPlayer::new(window_width, window_height, "momo preview")
        .map_err(|e| -> BoxError { format!("VideoPlayer creation failed: {e}").into() })?;

    player
        .play()
        .map_err(|e| -> BoxError { format!("VideoPlayer play failed: {e}").into() })?;

    loop {
        // Sora 接続終了の検知
        if shutdown_rx.try_recv().is_ok() {
            break;
        }

        // チャネルからフレームを受信 (ノンブロッキング)
        while let Ok(frame) = frame_rx.try_recv() {
            let _ = player.enqueue_video_i420(
                &frame.y,
                &frame.u,
                &frame.v,
                frame.width,
                frame.height,
                frame.pts_us,
            );
        }

        // SDL3 イベント処理 + レンダリング
        match player.poll_events() {
            Ok(true) => {}
            Ok(false) => {
                // ウィンドウが閉じられた。Sora 接続は継続。
                break;
            }
            Err(_) => break,
        }

        std::thread::sleep(Duration::from_millis(1));
    }

    raw_player::quit();
    Ok(())
}
