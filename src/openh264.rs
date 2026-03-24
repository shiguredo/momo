//! OpenH264 エンコーダー/デコーダーの WebRTC 統合
//!
//! shiguredo_openh264 の Encoder/Decoder を WebRTC の VideoEncoderFactory/VideoDecoderFactory で
//! ラップし、OpenH264 による H.264 エンコーディング/デコーディングを提供する。

use shiguredo_openh264::{
    DecodedFrame, EncodeOptions, Encoder, EncoderConfig, Openh264Library, Profile,
};
use shiguredo_webrtc::{
    CodecSpecificInfo, EncodedImage, EncodedImageBuffer, EncodedImageRef, EnvironmentRef,
    H264PacketizationMode, I420Buffer, SdpVideoFormat, SdpVideoFormatRef, VideoCodecRef,
    VideoCodecStatus, VideoDecoderDecodedImageCallbackPtr, VideoDecoderDecoderInfo,
    VideoDecoderFactory, VideoDecoderFactoryHandler, VideoDecoderHandler, VideoDecoderSettingsRef,
    VideoEncoderEncodedImageCallbackPtr, VideoEncoderEncodedImageCallbackRef,
    VideoEncoderEncoderInfo, VideoEncoderFactory, VideoEncoderFactoryHandler, VideoEncoderHandler,
    VideoEncoderRateControlParametersRef, VideoEncoderSettingsRef, VideoFrame as WebrtcVideoFrame,
    VideoFrameRef, VideoFrameType, VideoFrameTypeVectorRef,
};
use std::path::Path;
use tracing::{error, info, warn};

use crate::error::BoxError;

/// OpenH264 ライブラリを読み込む
pub(crate) fn load_openh264(path: &str) -> Result<Openh264Library, BoxError> {
    let lib = Openh264Library::load(Path::new(path))
        .map_err(|e| -> BoxError { format!("OpenH264 library load failed: {e}").into() })?;
    info!(
        target: "openh264",
        path = path,
        version = %lib.runtime_version(),
        "OpenH264 library loaded"
    );
    Ok(lib)
}

// ─── エンコーダー ──────────────────────────────────────────────────────────────

/// OpenH264 エンコーダーファクトリ
struct Openh264EncoderFactory {
    lib: Openh264Library,
    builtin: VideoEncoderFactory,
}

// SAFETY: VideoEncoderFactory は C++ unique_ptr をラップしており !Send だが、
// WebRTC のエンコーダーファクトリコールバックは signaling スレッドからのみ呼ばれるため安全。
unsafe impl Send for Openh264EncoderFactory {}

impl VideoEncoderFactoryHandler for Openh264EncoderFactory {
    fn get_supported_formats(&mut self) -> Vec<SdpVideoFormat> {
        // Constrained Baseline Profile を優先
        let mut format = SdpVideoFormat::new("H264");
        {
            let mut params = format.parameters_mut();
            params.set("profile-level-id", "42e01f");
            params.set("level-asymmetry-allowed", "1");
            params.set("packetization-mode", "1");
        }

        let mut formats = vec![format];

        // ビルトインのフォーマットも追加 (H.264 以外)
        for f in self.builtin.get_supported_formats() {
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
            return None;
        }

        info!(target: "openh264", "creating OpenH264 encoder");
        Some(Box::new(Openh264H264Encoder {
            lib: self.lib.clone(),
            encoder: None,
            callback: None,
            width: 0,
            height: 0,
        }))
    }
}

/// OpenH264 エンコーダーを含む VideoEncoderFactory を作成する
pub(crate) fn create_openh264_encoder_factory(lib: &Openh264Library) -> VideoEncoderFactory {
    let factory = Openh264EncoderFactory {
        lib: lib.clone(),
        builtin: VideoEncoderFactory::builtin(),
    };
    VideoEncoderFactory::new_with_handler(Box::new(factory))
}

/// OpenH264 H.264 エンコーダーの内部状態
struct Openh264H264Encoder {
    lib: Openh264Library,
    encoder: Option<Encoder>,
    callback: Option<VideoEncoderEncodedImageCallbackPtr>,
    width: u32,
    height: u32,
}

// SAFETY: Encoder は Send を実装しており、単一スレッドから使用される (WebRTC エンコーダスレッド)
unsafe impl Send for Openh264H264Encoder {}

impl VideoEncoderHandler for Openh264H264Encoder {
    fn init_encode(
        &mut self,
        codec_settings: VideoCodecRef<'_>,
        _settings: VideoEncoderSettingsRef<'_>,
    ) -> VideoCodecStatus {
        let width = codec_settings.width() as usize;
        let height = codec_settings.height() as usize;
        let bitrate_bps = (codec_settings.start_bitrate_kbps() * 1000) as usize;
        let fps = codec_settings.max_framerate() as usize;

        info!(
            target: "openh264",
            width,
            height,
            bitrate_bps,
            fps,
            "initializing OpenH264 encoder"
        );

        let mut config = EncoderConfig::new(width, height, bitrate_bps, fps, 1);
        config.profile = Some(Profile::ConstrainedBaseline);

        match Encoder::new(self.lib.clone(), config) {
            Ok(encoder) => {
                self.encoder = Some(encoder);
                self.width = width as u32;
                self.height = height as u32;
                info!(target: "openh264", "OpenH264 encoder initialized");
                VideoCodecStatus::Ok
            }
            Err(e) => {
                error!(target: "openh264", error = %e, "OpenH264 encoder init failed");
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

        let force_keyframe = frame_types
            .as_ref()
            .and_then(|ft| ft.get(0))
            .is_some_and(|t| t == VideoFrameType::Key);

        let rtp_timestamp = frame.rtp_timestamp();

        // I420 バッファからプレーンデータを取得
        let buffer = frame.buffer();
        let y_raw = buffer.y_data();
        let u_raw = buffer.u_data();
        let v_raw = buffer.v_data();
        let width = self.width as usize;
        let height = self.height as usize;
        let stride_y = buffer.stride_y() as usize;
        let stride_u = buffer.stride_u() as usize;
        let stride_v = buffer.stride_v() as usize;

        // OpenH264 はストライド = 幅を前提とするため、必要に応じてパック
        let (y, u, v) = if stride_y == width && stride_u == width / 2 && stride_v == width / 2 {
            (y_raw.to_vec(), u_raw.to_vec(), v_raw.to_vec())
        } else {
            let mut y = Vec::with_capacity(width * height);
            for row in 0..height {
                let start = row * stride_y;
                y.extend_from_slice(&y_raw[start..start + width]);
            }
            let mut u = Vec::with_capacity(width / 2 * height / 2);
            for row in 0..height / 2 {
                let start = row * stride_u;
                u.extend_from_slice(&u_raw[start..start + width / 2]);
            }
            let mut v = Vec::with_capacity(width / 2 * height / 2);
            for row in 0..height / 2 {
                let start = row * stride_v;
                v.extend_from_slice(&v_raw[start..start + width / 2]);
            }
            (y, u, v)
        };

        let options = EncodeOptions {
            force_idr: force_keyframe,
        };

        match encoder.encode(&y, &u, &v, &options) {
            Ok(Some(encoded)) => {
                if let Some(ref callback) = self.callback {
                    report_encoded_frame(
                        callback,
                        &encoded,
                        rtp_timestamp,
                        self.width,
                        self.height,
                    );
                }
                VideoCodecStatus::Ok
            }
            Ok(None) => {
                // スキップフレーム
                VideoCodecStatus::Ok
            }
            Err(e) => {
                warn!(target: "openh264", error = %e, "encode error");
                VideoCodecStatus::Error
            }
        }
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
        info!(target: "openh264", "OpenH264 encoder released");
        VideoCodecStatus::Ok
    }

    fn set_rates(&mut self, parameters: VideoEncoderRateControlParametersRef<'_>) {
        if let Some(ref mut encoder) = self.encoder {
            let bitrate_bps = parameters.target_bitrate_sum_bps() as usize;
            if let Err(e) = encoder.set_bitrate(bitrate_bps) {
                warn!(target: "openh264", error = %e, "set_bitrate failed");
            }
        }
    }

    fn get_encoder_info(&mut self) -> VideoEncoderEncoderInfo {
        let mut info = VideoEncoderEncoderInfo::new();
        info.set_implementation_name("OpenH264");
        info.set_is_hardware_accelerated(false);
        info
    }
}

/// エンコード結果を WebRTC コールバックに報告する
fn report_encoded_frame(
    callback: &VideoEncoderEncodedImageCallbackPtr,
    encoded: &shiguredo_openh264::EncodedFrame,
    rtp_timestamp: u32,
    width: u32,
    height: u32,
) {
    // SPS/PPS + スライスデータを Annex B 形式で結合
    let mut annex_b = Vec::new();
    for sps in &encoded.sps_list {
        annex_b.extend_from_slice(&[0, 0, 0, 1]);
        annex_b.extend_from_slice(sps);
    }
    for pps in &encoded.pps_list {
        annex_b.extend_from_slice(&[0, 0, 0, 1]);
        annex_b.extend_from_slice(pps);
    }
    annex_b.extend_from_slice(&encoded.data);

    let is_idr = matches!(encoded.frame_type, shiguredo_openh264::FrameType::Idr);

    let buffer = EncodedImageBuffer::from_bytes(&annex_b);
    let mut image = EncodedImage::new();
    image.set_encoded_data(&buffer);
    image.set_rtp_timestamp(rtp_timestamp);
    image.set_encoded_width(width);
    image.set_encoded_height(height);
    image.set_frame_type(if is_idr {
        VideoFrameType::Key
    } else {
        VideoFrameType::Delta
    });

    let mut codec_info = CodecSpecificInfo::new();
    codec_info.set_codec_type(shiguredo_webrtc::VideoCodecType::H264);
    codec_info.set_h264_packetization_mode(H264PacketizationMode::NonInterleaved);
    codec_info.set_h264_idr_frame(is_idr);

    // SAFETY: callback ポインタは register_encode_complete_callback で渡されたもので、
    // release が呼ばれるまで有効である。
    unsafe {
        callback.on_encoded_image(image.as_ref(), Some(codec_info.as_ref()));
    }
}

// ─── デコーダー ──────────────────────────────────────────────────────────────

/// OpenH264 デコーダーファクトリ
struct Openh264DecoderFactory {
    lib: Openh264Library,
    builtin: VideoDecoderFactory,
}

// SAFETY: VideoDecoderFactory は C++ unique_ptr をラップしており !Send だが、
// WebRTC のデコーダーファクトリコールバックは signaling スレッドからのみ呼ばれるため安全。
unsafe impl Send for Openh264DecoderFactory {}

impl VideoDecoderFactoryHandler for Openh264DecoderFactory {
    fn get_supported_formats(&mut self) -> Vec<SdpVideoFormat> {
        // Constrained Baseline Profile
        let mut format = SdpVideoFormat::new("H264");
        {
            let mut params = format.parameters_mut();
            params.set("profile-level-id", "42e01f");
            params.set("level-asymmetry-allowed", "1");
            params.set("packetization-mode", "1");
        }

        let mut formats = vec![format];

        // ビルトインのフォーマットも追加 (H.264 以外)
        for f in self.builtin.get_supported_formats() {
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
    ) -> Option<Box<dyn VideoDecoderHandler>> {
        let name = format.name().unwrap_or_default();
        if name != "H264" {
            return None;
        }

        info!(target: "openh264", "creating OpenH264 decoder");
        Some(Box::new(Openh264H264Decoder {
            lib: self.lib.clone(),
            decoder: None,
            callback: None,
        }))
    }
}

/// OpenH264 デコーダーを含む VideoDecoderFactory を作成する
pub(crate) fn create_openh264_decoder_factory(lib: &Openh264Library) -> VideoDecoderFactory {
    let factory = Openh264DecoderFactory {
        lib: lib.clone(),
        builtin: VideoDecoderFactory::builtin(),
    };
    VideoDecoderFactory::new_with_handler(Box::new(factory))
}

/// OpenH264 H.264 デコーダーの内部状態
struct Openh264H264Decoder {
    lib: Openh264Library,
    decoder: Option<shiguredo_openh264::Decoder>,
    callback: Option<VideoDecoderDecodedImageCallbackPtr>,
}

// SAFETY: Decoder は Send を実装しており、単一スレッドから使用される (WebRTC デコーダスレッド)
unsafe impl Send for Openh264H264Decoder {}

impl VideoDecoderHandler for Openh264H264Decoder {
    fn configure(&mut self, _settings: VideoDecoderSettingsRef<'_>) -> bool {
        info!(target: "openh264", "initializing OpenH264 decoder");

        match shiguredo_openh264::Decoder::new(self.lib.clone()) {
            Ok(decoder) => {
                self.decoder = Some(decoder);
                info!(target: "openh264", "OpenH264 decoder initialized");
                true
            }
            Err(e) => {
                error!(target: "openh264", error = %e, "OpenH264 decoder init failed");
                false
            }
        }
    }

    fn decode(
        &mut self,
        input_image: EncodedImageRef<'_>,
        _render_time_ms: i64,
    ) -> VideoCodecStatus {
        let Some(ref mut decoder) = self.decoder else {
            return VideoCodecStatus::Uninitialized;
        };

        let Some(data) = input_image.encoded_data() else {
            return VideoCodecStatus::Error;
        };
        let rtp_timestamp = input_image.rtp_timestamp();

        match decoder.decode(data.data()) {
            Ok(Some(decoded)) => {
                if let Some(ref callback) = self.callback {
                    deliver_decoded_frame(callback, &decoded, rtp_timestamp);
                }
                VideoCodecStatus::Ok
            }
            Ok(None) => {
                // デコード結果がまだない (バッファリング中)
                VideoCodecStatus::Ok
            }
            Err(e) => {
                warn!(target: "openh264", error = %e, "decode error");
                VideoCodecStatus::Error
            }
        }
    }

    fn register_decode_complete_callback(
        &mut self,
        callback: Option<VideoDecoderDecodedImageCallbackPtr>,
    ) -> VideoCodecStatus {
        self.callback = callback;
        VideoCodecStatus::Ok
    }

    fn release(&mut self) -> VideoCodecStatus {
        self.decoder = None;
        self.callback = None;
        info!(target: "openh264", "OpenH264 decoder released");
        VideoCodecStatus::Ok
    }

    fn get_decoder_info(&mut self) -> VideoDecoderDecoderInfo {
        let mut info = VideoDecoderDecoderInfo::new();
        info.set_implementation_name("OpenH264");
        info.set_is_hardware_accelerated(false);
        info
    }
}

/// デコード結果を WebRTC コールバックに報告する
fn deliver_decoded_frame(
    callback: &VideoDecoderDecodedImageCallbackPtr,
    decoded: &DecodedFrame,
    rtp_timestamp: u32,
) {
    let width = decoded.width() as i32;
    let height = decoded.height() as i32;

    let mut i420 = I420Buffer::new(width, height);
    let y_stride = decoded.y_stride();
    let u_stride = decoded.u_stride();
    let v_stride = decoded.v_stride();
    let w = width as usize;
    let h = height as usize;

    // I420Buffer にコピー (ストライドが異なる場合があるため行ごとにコピー)
    let dst_stride_y = i420.stride_y() as usize;
    let dst_stride_u = i420.stride_u() as usize;
    let dst_stride_v = i420.stride_v() as usize;
    let half_w = w / 2;

    {
        let dst_y = i420.y_data_mut();
        let src_y = decoded.y_plane();
        for row in 0..h {
            let src_start = row * y_stride;
            let dst_start = row * dst_stride_y;
            dst_y[dst_start..dst_start + w].copy_from_slice(&src_y[src_start..src_start + w]);
        }
    }
    {
        let dst_u = i420.u_data_mut();
        let src_u = decoded.u_plane();
        for row in 0..h / 2 {
            let src_start = row * u_stride;
            let dst_start = row * dst_stride_u;
            dst_u[dst_start..dst_start + half_w]
                .copy_from_slice(&src_u[src_start..src_start + half_w]);
        }
    }
    {
        let dst_v = i420.v_data_mut();
        let src_v = decoded.v_plane();
        for row in 0..h / 2 {
            let src_start = row * v_stride;
            let dst_start = row * dst_stride_v;
            dst_v[dst_start..dst_start + half_w]
                .copy_from_slice(&src_v[src_start..src_start + half_w]);
        }
    }

    let timestamp_us = rtp_timestamp as i64 * 1000 / 90;
    let frame = WebrtcVideoFrame::from_i420(&i420, timestamp_us, rtp_timestamp);

    // SAFETY: callback ポインタは register_decode_complete_callback で渡されたもので、
    // release が呼ばれるまで有効である。
    unsafe {
        callback.decoded(frame.as_ref());
    }
}

// ─── sora_sdk 用 VideoCodecCapability 実装 ──────────────────────────────────

#[cfg(feature = "sora")]
pub(crate) mod sora_capability {
    use std::collections::HashMap;

    use shiguredo_openh264::Openh264Library;
    use shiguredo_webrtc::{
        SdpVideoFormat, VideoCodecType, VideoDecoderHandler, VideoEncoderHandler,
    };
    use sora_sdk::{CodecDirection, VideoCodecCapability, VideoCodecImplementation};

    use super::{Openh264H264Decoder, Openh264H264Encoder};

    /// OpenH264 の VideoCodecCapability 実装
    pub(crate) struct Openh264VideoCodecCapability {
        pub lib: Openh264Library,
    }

    impl VideoCodecCapability for Openh264VideoCodecCapability {
        fn get_implementation(&self) -> VideoCodecImplementation {
            VideoCodecImplementation::new("openh264", "Cisco OpenH264")
        }

        fn is_supported(&self, _direction: CodecDirection, codec_type: VideoCodecType) -> bool {
            // H.264 のエンコード/デコード両方に対応
            codec_type == VideoCodecType::H264
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
            Some(Box::new(Openh264H264Encoder {
                lib: self.lib.clone(),
                encoder: None,
                callback: None,
                width: 0,
                height: 0,
            }))
        }

        fn create_video_decoder(
            &self,
            format: &SdpVideoFormat,
        ) -> Option<Box<dyn VideoDecoderHandler>> {
            let name = format.name().ok()?;
            if name != "H264" {
                return None;
            }
            Some(Box::new(Openh264H264Decoder {
                lib: self.lib.clone(),
                decoder: None,
                callback: None,
            }))
        }
    }
}
