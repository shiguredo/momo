use std::time::Duration;

use shiguredo_webrtc::{
    AdaptedVideoTrackSource, AudioDeviceModule, AudioDeviceModuleAudioLayer, Environment,
    I420Buffer, VideoFrame as WebrtcVideoFrame,
};

use crate::error::{BoxError, wrtc_err};

// ─── BGRA → I420 変換 ─────────────────────────────────────────────────────────

/// BGRA バイト列を I420 バッファに変換する (BT.601 限定レンジ)
///
/// raden の Prgb32 はメモリ上 [B, G, R, A] の順で格納されている。
fn bgra_to_i420(bgra: &[u8], width: i32, height: i32) -> I420Buffer {
    use shiguredo_webrtc::ffi::*;

    let buf = I420Buffer::new(width, height);

    // SAFETY: I420Buffer::new が割り当てた参照カウント付きバッファを排他的に所有している。
    // `buf` が生きている間、派生したポインタは有効である。
    unsafe {
        let refcounted = buf.as_refcounted_ptr();
        let raw = webrtc_I420Buffer_refcounted_get(refcounted);

        let y_ptr = webrtc_I420Buffer_MutableDataY(raw);
        let u_ptr = webrtc_I420Buffer_MutableDataU(raw);
        let v_ptr = webrtc_I420Buffer_MutableDataV(raw);
        let stride_y = webrtc_I420Buffer_StrideY(raw) as usize;
        let stride_u = webrtc_I420Buffer_StrideU(raw) as usize;
        let stride_v = webrtc_I420Buffer_StrideV(raw) as usize;

        let w = width as usize;
        let h = height as usize;

        for row in 0..h {
            for col in 0..w {
                let px = (row * w + col) * 4;
                let b = bgra[px] as i32;
                let g = bgra[px + 1] as i32;
                let r = bgra[px + 2] as i32;

                // BT.601 限定レンジ Y
                let y = 16 + ((66 * r + 129 * g + 25 * b) >> 8);
                *y_ptr.add(row * stride_y + col) = y.clamp(16, 235) as u8;

                // U/V は 2×2 ブロックに 1 サンプル
                if row % 2 == 0 && col % 2 == 0 {
                    let u = 128 + ((-38 * r - 74 * g + 112 * b) >> 8);
                    let v = 128 + ((112 * r - 94 * g - 18 * b) >> 8);
                    *u_ptr.add((row / 2) * stride_u + col / 2) = u.clamp(16, 240) as u8;
                    *v_ptr.add((row / 2) * stride_v + col / 2) = v.clamp(16, 240) as u8;
                }
            }
        }
    }

    buf
}

/// 色相 (0.0..360.0) から RGB (各 0..255) に変換する
fn hue_to_rgb(hue: f32) -> (u8, u8, u8) {
    let h = hue / 60.0;
    let i = h as i32;
    let f = h - i as f32;
    let q = ((1.0 - f) * 255.0) as u8;
    let t = (f * 255.0) as u8;
    let v = 255u8;
    match i % 6 {
        0 => (v, t, 0),
        1 => (q, v, 0),
        2 => (0, v, t),
        3 => (0, q, v),
        4 => (t, 0, v),
        _ => (v, 0, q),
    }
}

// ─── ダミー音声デバイス ────────────────────────────────────────────────────────

/// WebRTC 組み込みのダミー ADM を作成する (無音を生成)
pub(crate) fn create_dummy_adm(env: &Environment) -> Result<AudioDeviceModule, BoxError> {
    AudioDeviceModule::new(env, AudioDeviceModuleAudioLayer::Dummy).map_err(wrtc_err)
}

// ─── フェイク映像スレッド ─────────────────────────────────────────────────────

/// raden でアニメーションフレームを生成し AdaptedVideoTrackSource に供給するスレッドを起動する
pub(crate) fn start_fake_video_thread(
    source: AdaptedVideoTrackSource,
    width: i32,
    height: i32,
    fps: u32,
    #[cfg(feature = "preview")] preview_tx: Option<
        std::sync::mpsc::SyncSender<crate::preview::PreviewFrame>,
    >,
) {
    std::thread::Builder::new()
        .name("fake-video".to_string())
        .spawn(move || {
            // adapt_frame / on_frame は &mut self を必要とする
            let mut source = source;
            let mut image =
                raden::Image::new(width as u32, height as u32, raden::PixelFormat::Prgb32);
            let mut runtime = raden::PipelineRuntime::new();
            let frame_duration = Duration::from_nanos(1_000_000_000 / fps as u64);
            let mut frame_idx = 0u64;

            loop {
                // raden でフレームを描画
                {
                    let mut ctx = raden::Context::new(&mut image, &mut runtime);

                    // 色相サイクル背景
                    let hue = (frame_idx % 360) as f32;
                    let (r, g, b) = hue_to_rgb(hue);
                    ctx.set_fill_style(raden::Rgba32::rgb(r, g, b));
                    ctx.fill_all();

                    // 移動する白い矩形
                    let max_x = (width as u64).saturating_sub(64);
                    let rect_x = if max_x > 0 {
                        (frame_idx * 2 % max_x) as f64
                    } else {
                        0.0
                    };
                    ctx.set_fill_style(raden::Rgba32::rgb(255, 255, 255));
                    ctx.fill_rect(&raden::Rect::new(rect_x, 10.0, 64.0, 64.0));
                }

                // BGRA → I420 変換してフレームを供給
                let timestamp_us = shiguredo_webrtc::time_millis() * 1000;
                let adapt_result = source.adapt_frame(width, height, timestamp_us);
                if adapt_result.applied {
                    let i420 = bgra_to_i420(image.data(), width, height);

                    // プレビューウィンドウへフレームを送信 (ベストエフォート)
                    #[cfg(feature = "preview")]
                    if let Some(ref tx) = preview_tx {
                        let preview_frame = crate::preview::extract_preview_frame(
                            &i420,
                            width,
                            height,
                            timestamp_us,
                        );
                        let _ = tx.try_send(preview_frame);
                    }

                    let frame = WebrtcVideoFrame::from_i420(
                        &i420,
                        timestamp_us,
                        (timestamp_us * 90 / 1000) as u32,
                    );
                    source.on_frame(&frame);
                }

                frame_idx += 1;
                std::thread::sleep(frame_duration);
            }
        })
        .expect("フェイク映像スレッドの起動に失敗");
}
