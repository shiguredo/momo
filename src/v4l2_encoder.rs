//! V4L2 H.264 ハードウェアエンコーダーの WebRTC 統合 (Raspberry Pi 専用)
//!
//! shiguredo_v4l2 の H264Encoder を WebRTC の VideoEncoderFactory で
//! ラップし、ハードウェアアクセラレーションによる H.264 エンコーディングを提供する。

use shiguredo_v4l2::v4l2_m2m::{EncodedFrame, EncoderConfig, H264Encoder, InputFrame, InputMemory};

#[cfg(feature = "raspberrypi")]
use crate::libcamera::DmaBufMap;
use shiguredo_webrtc::{
    CodecSpecificInfo, EncodedImage, EncodedImageBuffer, EnvironmentRef, H264PacketizationMode,
    SdpVideoFormat, SdpVideoFormatRef, VideoCodecRef, VideoCodecStatus,
    VideoEncoderEncodedImageCallbackPtr, VideoEncoderEncodedImageCallbackRef,
    VideoEncoderEncoderInfo, VideoEncoderFactory, VideoEncoderFactoryHandler, VideoEncoderHandler,
    VideoEncoderRateControlParametersRef, VideoEncoderSettingsRef, VideoFrameRef, VideoFrameType,
    VideoFrameTypeVectorRef,
};
use tracing::{error, info, warn};

/// V4L2 H.264 エンコーダーファクトリ
struct V4l2EncoderFactory {
    builtin_for_formats: VideoEncoderFactory,
    dmabuf_map: Option<DmaBufMap>,
}

// SAFETY: VideoEncoderFactory は C++ unique_ptr をラップしており !Send だが、
// WebRTC のエンコーダーファクトリコールバックは signaling スレッドからのみ呼ばれるため安全。
unsafe impl Send for V4l2EncoderFactory {}

impl VideoEncoderFactoryHandler for V4l2EncoderFactory {
    fn get_supported_formats(&mut self) -> Vec<SdpVideoFormat> {
        // Constrained Baseline Profile (42e0) を優先
        let mut format = SdpVideoFormat::new("H264");
        {
            let mut params = format.parameters_mut();
            params.set("profile-level-id", "42e01f");
            params.set("level-asymmetry-allowed", "1");
            params.set("packetization-mode", "1");
        }

        let mut formats = vec![format];

        // ビルトインのフォーマットも追加（VP8, VP9 など H.264 以外）
        for f in self.builtin_for_formats.get_supported_formats() {
            let name = f.name().unwrap_or_default();
            if name != "H264" {
                formats.push(f);
            }
        }

        formats
    }

    fn create(
        &mut self,
        _env: EnvironmentRef<'_>,
        format: SdpVideoFormatRef<'_>,
    ) -> Option<Box<dyn VideoEncoderHandler>> {
        let name = format.name().unwrap_or_default();
        if name != "H264" {
            // H.264 以外はビルトインに委譲
            // ビルトインは VideoEncoder を返すが Handler を返す必要がある
            // ビルトインのファクトリに委譲できないため None を返す
            return None;
        }

        info!(target: "v4l2", "creating V4L2 H.264 encoder");
        Some(Box::new(V4l2H264Encoder {
            encoder: None,
            callback: None,
            width: 0,
            height: 0,
            dmabuf_map: self.dmabuf_map.clone(),
        }))
    }
}

/// V4L2 H.264 エンコーダーを含む VideoEncoderFactory を作成する
///
/// ビルトインのソフトウェアエンコーダーの代わりに、V4L2 M2M デバイスによる
/// ハードウェア H.264 エンコーディングを使用する。
/// H.264 以外のコーデックはビルトインファクトリにフォールバックする。
pub(crate) fn create_v4l2_encoder_factory(dmabuf_map: Option<DmaBufMap>) -> VideoEncoderFactory {
    let factory = V4l2EncoderFactory {
        builtin_for_formats: VideoEncoderFactory::builtin(),
        dmabuf_map,
    };
    VideoEncoderFactory::new_with_handler(Box::new(factory))
}

/// V4L2 H.264 エンコーダーの内部状態
struct V4l2H264Encoder {
    encoder: Option<H264Encoder>,
    callback: Option<VideoEncoderEncodedImageCallbackPtr>,
    width: u32,
    height: u32,
    dmabuf_map: Option<DmaBufMap>,
}

// SAFETY: H264Encoder は単一スレッドから使用される (WebRTC エンコーダスレッド)
unsafe impl Send for V4l2H264Encoder {}

impl VideoEncoderHandler for V4l2H264Encoder {
    fn init_encode(
        &mut self,
        codec_settings: VideoCodecRef<'_>,
        _settings: VideoEncoderSettingsRef<'_>,
    ) -> VideoCodecStatus {
        let width = codec_settings.width() as u32;
        let height = codec_settings.height() as u32;
        let bitrate_bps = codec_settings.start_bitrate_kbps() * 1000;

        info!(
            target: "v4l2",
            width,
            height,
            bitrate_bps,
            "initializing V4L2 H.264 encoder"
        );

        let mut config = EncoderConfig::new(width, height, bitrate_bps);
        if self.dmabuf_map.is_some() {
            config.input_memory = InputMemory::DmaBuf;
        }
        match H264Encoder::new(config) {
            Ok(encoder) => {
                self.encoder = Some(encoder);
                self.width = width;
                self.height = height;
                info!(target: "v4l2", "V4L2 H.264 encoder initialized");
                VideoCodecStatus::Ok
            }
            Err(e) => {
                error!(target: "v4l2", error = %e, "V4L2 H.264 encoder init failed");
                VideoCodecStatus::FallbackSoftware
            }
        }
    }

    fn encode(
        &mut self,
        frame: VideoFrameRef<'_>,
        frame_types: Option<VideoFrameTypeVectorRef<'_>>,
    ) -> VideoCodecStatus {
        let Some(ref mut encoder) = self.encoder else {
            return VideoCodecStatus::Uninitialized;
        };

        // キーフレーム要求の確認
        let force_keyframe = frame_types
            .as_ref()
            .and_then(|ft| ft.get(0))
            .is_some_and(|t| t == VideoFrameType::Key);

        let timestamp_us = frame.timestamp_us();
        let rtp_timestamp = frame.rtp_timestamp();

        // DmaBuf マップから DMA-BUF エントリを取得 (ネイティブモード)
        let dmabuf_entry = self.dmabuf_map.as_ref().and_then(|map| {
            let mut guard = map.lock().unwrap();
            guard.remove(&timestamp_us)
        });

        // V4L2 エンコード
        let encoded = if let Some(entry) = dmabuf_entry {
            // ── ネイティブモード: DMA-BUF fd をゼロコピーで渡す ──
            let result = encoder.encode(
                InputFrame::DmaBuf {
                    fd: entry.fd,
                    bytesused: entry.bytesused,
                    length: entry.length,
                },
                timestamp_us,
                force_keyframe,
            );
            // エンコード完了を libcamera に通知 (requeue を許可)
            let _ = entry.done_tx.send(());
            match result {
                Ok(f) => f,
                Err(e) => {
                    warn!(target: "v4l2", error = %e, "encode error (dmabuf)");
                    return VideoCodecStatus::Error;
                }
            }
        } else {
            // ── 通常モード: I420 データをコピーして渡す ──
            let buffer = frame.buffer();
            let Some(i420) = buffer.as_i420() else {
                warn!(target: "v4l2", "encode: frame buffer is not I420");
                return VideoCodecStatus::Error;
            };
            let y = i420.y_data();
            let u = i420.u_data();
            let v = i420.v_data();
            let width = self.width as usize;
            let height = self.height as usize;

            let yuv_size = width * height * 3 / 2;
            let mut i420_data = Vec::with_capacity(yuv_size);
            let stride_y = i420.stride_y() as usize;
            let stride_u = i420.stride_u() as usize;
            let stride_v = i420.stride_v() as usize;

            // Y plane
            for row in 0..height {
                let start = row * stride_y;
                let end = start + width;
                i420_data.extend_from_slice(&y[start..end]);
            }
            // U plane
            for row in 0..height / 2 {
                let start = row * stride_u;
                let end = start + width / 2;
                i420_data.extend_from_slice(&u[start..end]);
            }
            // V plane
            for row in 0..height / 2 {
                let start = row * stride_v;
                let end = start + width / 2;
                i420_data.extend_from_slice(&v[start..end]);
            }

            match encoder.encode(InputFrame::I420(&i420_data), timestamp_us, force_keyframe) {
                Ok(f) => f,
                Err(e) => {
                    warn!(target: "v4l2", error = %e, "encode error");
                    return VideoCodecStatus::Error;
                }
            }
        };

        // WebRTC に通知
        if let Some(ref callback) = self.callback {
            report_encoded_frame(callback, &encoded, rtp_timestamp, self.width, self.height);
        }

        VideoCodecStatus::Ok
    }

    fn register_encode_complete_callback(
        &mut self,
        callback: Option<VideoEncoderEncodedImageCallbackRef<'_>>,
    ) -> VideoCodecStatus {
        self.callback =
            callback.map(|cb| unsafe { VideoEncoderEncodedImageCallbackPtr::from_ref(cb) });
        VideoCodecStatus::Ok
    }

    fn release(&mut self) -> VideoCodecStatus {
        self.encoder = None;
        self.callback = None;
        info!(target: "v4l2", "V4L2 H.264 encoder released");
        VideoCodecStatus::Ok
    }

    fn set_rates(&mut self, parameters: VideoEncoderRateControlParametersRef<'_>) {
        if let Some(ref mut encoder) = self.encoder {
            let bitrate_bps = parameters.target_bitrate_sum_bps();
            if let Err(e) = encoder.set_bitrate(bitrate_bps) {
                warn!(target: "v4l2", error = %e, "set_bitrate failed");
            }
        }
    }

    fn get_encoder_info(&mut self) -> VideoEncoderEncoderInfo {
        let mut info = VideoEncoderEncoderInfo::new();
        info.set_implementation_name("V4L2 H.264");
        info.set_is_hardware_accelerated(true);
        info
    }
}

// ─── sora_sdk 用 VideoCodecCapability 実装 ──────────────────────────────────

#[cfg(feature = "sora")]
pub(crate) mod sora_capability {
    use std::collections::HashMap;

    use shiguredo_webrtc::{
        SdpVideoFormat, VideoCodecType, VideoDecoderHandler, VideoEncoderHandler,
    };
    use sora_sdk::video_codec_capability::{
        CodecDirection, VideoCodecCapability, VideoCodecImplementation,
    };

    use super::V4l2H264Encoder;

    /// V4L2 H.264 ハードウェアエンコーダーの VideoCodecCapability 実装
    pub(crate) struct V4l2VideoCodecCapability {
        pub dmabuf_map: Option<super::DmaBufMap>,
    }

    impl VideoCodecCapability for V4l2VideoCodecCapability {
        fn get_implementation(&self) -> VideoCodecImplementation {
            VideoCodecImplementation::new("v4l2", "V4L2 M2M H.264 Hardware Encoder")
        }

        fn is_supported(&self, direction: CodecDirection, codec_type: VideoCodecType) -> bool {
            // H.264 エンコードのみ対応
            direction == CodecDirection::Encoder && codec_type == VideoCodecType::H264
        }

        fn resolve_sdp_format(
            &self,
            direction: CodecDirection,
            codec_type: VideoCodecType,
            _parameters: &HashMap<String, String>,
            _scalability_mode: Option<&str>,
        ) -> Option<SdpVideoFormat> {
            if !self.is_supported(direction, codec_type) {
                return None;
            }
            // Constrained Baseline Profile
            let mut format = SdpVideoFormat::new("H264");
            {
                let mut params = format.parameters_mut();
                params.set("profile-level-id", "42e01f");
                params.set("level-asymmetry-allowed", "1");
                params.set("packetization-mode", "1");
            }
            Some(format)
        }

        fn create_video_encoder(
            &self,
            format: &SdpVideoFormat,
        ) -> Option<Box<dyn VideoEncoderHandler>> {
            let name = format.name().ok()?;
            if name != "H264" {
                return None;
            }
            Some(Box::new(V4l2H264Encoder {
                encoder: None,
                callback: None,
                width: 0,
                height: 0,
                dmabuf_map: self.dmabuf_map.clone(),
            }))
        }

        fn create_video_decoder(
            &self,
            _format: &SdpVideoFormat,
        ) -> Option<Box<dyn VideoDecoderHandler>> {
            // V4L2 デコーダーは未対応
            None
        }
    }
}

/// エンコード結果を WebRTC コールバックに報告する
fn report_encoded_frame(
    callback: &VideoEncoderEncodedImageCallbackPtr,
    encoded: &EncodedFrame,
    rtp_timestamp: u32,
    width: u32,
    height: u32,
) {
    let buffer = EncodedImageBuffer::from_bytes(&encoded.data);
    let mut image = EncodedImage::new();
    image.set_encoded_data(&buffer);
    image.set_rtp_timestamp(rtp_timestamp);
    image.set_encoded_width(width);
    image.set_encoded_height(height);
    image.set_frame_type(if encoded.is_keyframe {
        VideoFrameType::Key
    } else {
        VideoFrameType::Delta
    });

    let mut codec_info = CodecSpecificInfo::new();
    codec_info.set_codec_type(shiguredo_webrtc::VideoCodecType::H264);
    codec_info.set_h264_packetization_mode(H264PacketizationMode::NonInterleaved);
    codec_info.set_h264_idr_frame(encoded.is_keyframe);

    // SAFETY: callback ポインタは register_encode_complete_callback で渡されたもので、
    // release が呼ばれるまで有効である。
    unsafe {
        callback.on_encoded_image(image.as_ref(), Some(codec_info.as_ref()));
    }
}
