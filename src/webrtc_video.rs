//! shiguredo_webrtc の `VideoFrame` 組み立てとビデオキャプチャの I420 変換

use shiguredo_video_device::PixelFormat;
use shiguredo_webrtc::{I420Buffer, VideoFrame, nv12_to_i420, yuy2_to_i420};

/// ビデオキャプチャのフレームを I420 に変換する。未対応フォーマットは `None`。
pub(crate) fn capture_frame_to_i420(
    pixel_format: PixelFormat,
    data: &[u8],
    stride: i32,
    stride_uv: i32,
    uv_data: Option<&[u8]>,
    width: i32,
    height: i32,
) -> Option<I420Buffer> {
    let mut out = I420Buffer::new(width, height);
    match pixel_format {
        PixelFormat::Nv12 => {
            let uv = uv_data.unwrap_or(&[]);
            let sy = out.stride_y();
            let su = out.stride_u();
            let sv = out.stride_v();
            let (y, u, v) = out.planes_mut();
            if nv12_to_i420(
                data, stride, uv, stride_uv, y, sy, u, su, v, sv, width, height,
            ) {
                Some(out)
            } else {
                None
            }
        }
        PixelFormat::Yuy2 => {
            let sy = out.stride_y();
            let su = out.stride_u();
            let sv = out.stride_v();
            let (y, u, v) = out.planes_mut();
            if yuy2_to_i420(data, stride, y, sy, u, su, v, sv, width, height) {
                Some(out)
            } else {
                None
            }
        }
        _ => None,
    }
}

/// I420 バッファとタイムスタンプから `VideoFrame` を構築する。
pub(crate) fn video_frame_from_i420(
    i420: &I420Buffer,
    timestamp_us: i64,
    rtp_timestamp: u32,
) -> VideoFrame {
    let vfb = i420.cast_to_video_frame_buffer();
    let mut builder = VideoFrame::builder(&vfb);
    builder
        .set_timestamp_us(timestamp_us)
        .set_rtp_timestamp(rtp_timestamp);
    builder.build()
}
