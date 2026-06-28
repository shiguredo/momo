//! V4L2 H.264 ハードウェアエンコーダーの WebRTC 統合 (Raspberry Pi 専用)
//!
//! shiguredo_v4l2 の H264Encoder を WebRTC の VideoEncoderFactory で
//! ラップし、ハードウェアアクセラレーションによる H.264 エンコーディングを提供する。

use std::sync::{Arc, Mutex};

use shiguredo_v4l2::v4l2_m2m::{
    EncodeInput, EncodedFrame, EncoderConfig, Error as V4l2Error, FnEncodeHandler, H264Encoder,
    Memory, Resolution,
};
use shiguredo_webrtc::{
    CodecSpecificInfo, EncodedImage, EncodedImageBuffer, EnvironmentRef, H264PacketizationMode,
    SdpVideoFormat, SdpVideoFormatRef, VideoCodecRef, VideoCodecStatus, VideoEncoder,
    VideoEncoderEncodedImageCallbackPtr, VideoEncoderEncodedImageCallbackRef,
    VideoEncoderEncodedImageCallbackResultError, VideoEncoderEncoderInfo, VideoEncoderFactory,
    VideoEncoderFactoryHandler, VideoEncoderHandler, VideoEncoderRateControlParametersRef,
    VideoEncoderSettingsRef, VideoFrameRef, VideoFrameType, VideoFrameTypeVectorRef, i420_copy,
};
use tracing::{error, info, warn};

use crate::libcamera::DmaBufMap;

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
    ) -> Option<VideoEncoder> {
        let name = format.name().unwrap_or_default();
        if name != "H264" {
            // H.264 以外はビルトインに委譲
            // ビルトインは VideoEncoder を返すが Handler を返す必要がある
            // ビルトインのファクトリに委譲できないため None を返す
            return None;
        }

        info!(target: "v4l2", "creating V4L2 H.264 encoder");
        Some(VideoEncoder::new_with_handler(Box::new(
            V4l2H264Encoder::new(self.dmabuf_map.clone()),
        )))
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

/// エンコーダーコールバックに渡す値
///
/// CAPTURE バッファが dequeue されるタイミングで V4L2 から戻ってくる。
/// dmabuf の done_tx は V4L2 が消費しきるまで生かしておくためここに保持する。
struct EncoderCallbackValue {
    rtp_timestamp: u32,
    width: u32,
    height: u32,
    /// dmabuf 入力時のみ Some。コールバックで `done_tx.send(())` を呼んで
    /// libcamera 側の requeue を解除する。
    dmabuf_done_tx: Option<std::sync::mpsc::SyncSender<()>>,
}

/// V4L2 エンコーダーの共有状態
///
/// V4L2 のコールバックは別スレッドから呼ばれるため、WebRTC 側のコールバックと
/// エンコーダー本体は Mutex で保護した共有状態に置く。
#[derive(Default)]
struct V4l2EncoderSharedState {
    encoder: Option<H264Encoder<FnEncodeHandler<EncoderCallbackValue>>>,
    callback: Option<VideoEncoderEncodedImageCallbackPtr>,
}

/// V4L2 のコールバックスレッドからエンコード結果を受け取って WebRTC に転送する
fn handle_v4l2_encode_callback(
    shared_state: &Arc<Mutex<V4l2EncoderSharedState>>,
    result: shiguredo_v4l2::v4l2_m2m::Result<EncodedFrame<EncoderCallbackValue>>,
) {
    let encoded = match result {
        Ok(encoded) => encoded,
        Err(err) => {
            warn!(target: "v4l2", error = %err, "encode callback error");
            return;
        }
    };

    // dmabuf 入力ならここで libcamera 側の requeue を解放する。
    // V4L2 コールバックが呼ばれた時点で V4L2 ハードウェアは入力 fd を
    // 消費しきっているため、libcamera にバッファを返して問題ない。
    if let Some(ref done_tx) = encoded.user_data().dmabuf_done_tx {
        let _ = done_tx.send(());
    }

    let Some(encoded_data) = encoded.data() else {
        warn!(target: "v4l2", "encoded frame has no MMAP data");
        return;
    };

    let buffer = EncodedImageBuffer::from_bytes(encoded_data);
    let mut image = EncodedImage::new();
    image.set_encoded_data(&buffer);
    image.set_rtp_timestamp(encoded.user_data().rtp_timestamp);
    image.set_encoded_width(encoded.user_data().width);
    image.set_encoded_height(encoded.user_data().height);
    image.set_frame_type(if encoded.is_keyframe() {
        VideoFrameType::Key
    } else {
        VideoFrameType::Delta
    });

    let mut codec_info = CodecSpecificInfo::new();
    codec_info.set_codec_type(shiguredo_webrtc::VideoCodecType::H264);
    codec_info.set_h264_packetization_mode(H264PacketizationMode::NonInterleaved);
    codec_info.set_h264_idr_frame(encoded.is_keyframe());

    let callback = {
        let guard = shared_state.lock().unwrap();
        guard.callback
    };
    let Some(callback) = callback else {
        return;
    };

    // SAFETY: callback ポインタは register_encode_complete_callback で渡されたもので、
    // release が呼ばれるまで有効である。
    let result = unsafe { callback.on_encoded_image(image.as_ref(), Some(codec_info.as_ref())) };
    if result.error() != VideoEncoderEncodedImageCallbackResultError::Ok {
        warn!(target: "v4l2", "on_encoded_image returned non-Ok status");
    }
}

/// V4L2 H.264 エンコーダーの内部状態
struct V4l2H264Encoder {
    shared_state: Arc<Mutex<V4l2EncoderSharedState>>,
    width: u32,
    height: u32,
    bitrate_bps: u32,
    dmabuf_map: Option<DmaBufMap>,
}

impl V4l2H264Encoder {
    fn new(dmabuf_map: Option<DmaBufMap>) -> Self {
        Self {
            shared_state: Arc::new(Mutex::new(V4l2EncoderSharedState::default())),
            width: 0,
            height: 0,
            bitrate_bps: 0,
            dmabuf_map,
        }
    }

    fn build_encoder(
        &self,
    ) -> Result<H264Encoder<FnEncodeHandler<EncoderCallbackValue>>, V4l2Error> {
        let mut config = EncoderConfig::new(self.width, self.height, self.bitrate_bps.max(1));
        if self.dmabuf_map.is_some() {
            config.input_memory = Memory::DmaBuf;
        }
        let shared_state = self.shared_state.clone();
        H264Encoder::new(
            config,
            FnEncodeHandler::new(move |result| {
                handle_v4l2_encode_callback(&shared_state, result);
            }),
        )
    }
}

impl VideoEncoderHandler for V4l2H264Encoder {
    fn init_encode(
        &mut self,
        codec_settings: VideoCodecRef<'_>,
        _settings: VideoEncoderSettingsRef<'_>,
    ) -> VideoCodecStatus {
        self.width = codec_settings.width().max(0) as u32;
        self.height = codec_settings.height().max(0) as u32;
        self.bitrate_bps = codec_settings.start_bitrate_kbps().saturating_mul(1000);

        info!(
            target: "v4l2",
            width = self.width,
            height = self.height,
            bitrate_bps = self.bitrate_bps,
            "initializing V4L2 H.264 encoder"
        );

        match self.build_encoder() {
            Ok(encoder) => {
                let mut guard = self.shared_state.lock().unwrap();
                guard.encoder = Some(encoder);
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

        if let Some(entry) = dmabuf_entry {
            // ── ネイティブモード: DMA-BUF fd をゼロコピーで渡す ──
            // entry の done_tx は V4L2 が消費しきるまで EncoderCallbackValue で生かしておく。
            let fd = entry.fd;
            let bytesused = entry.bytesused;
            let length = entry.length;
            let done_tx = entry.done_tx;
            let value = EncoderCallbackValue {
                rtp_timestamp,
                width: self.width,
                height: self.height,
                dmabuf_done_tx: Some(done_tx),
            };
            let mut guard = self.shared_state.lock().unwrap();
            let Some(encoder) = guard.encoder.as_mut() else {
                return VideoCodecStatus::Uninitialized;
            };
            return match encoder.encode(
                EncodeInput::DmaBuf {
                    fd,
                    bytesused,
                    length,
                },
                timestamp_us,
                force_keyframe,
                value,
            ) {
                Ok(()) => VideoCodecStatus::Ok,
                Err(V4l2Error::NoAvailableBuffer) => VideoCodecStatus::NoOutput,
                Err(e) => {
                    warn!(target: "v4l2", error = %e, "encode error (dmabuf)");
                    VideoCodecStatus::Error
                }
            };
        }

        // ── 通常モード: I420 データを MMAP バッファへコピー ──
        let buffer = frame.buffer();
        let Some(i420) = buffer.as_i420() else {
            warn!(target: "v4l2", "encode: frame buffer is not I420");
            return VideoCodecStatus::Error;
        };

        let mut fill = |dst: &mut [u8],
                        resolution: &Resolution,
                        _value: &EncoderCallbackValue|
         -> Option<usize> {
            let chroma_stride = resolution.stride.div_ceil(2);
            let chroma_height = resolution.height.div_ceil(2);
            let yuv_size = resolution.yuv420_size();
            if dst.len() < yuv_size {
                return None;
            }
            let y_size = (resolution.stride as usize) * (resolution.height as usize);
            let uv_size = (chroma_stride as usize) * (chroma_height as usize);
            let dst_stride_y = i32::try_from(resolution.stride).ok()?;
            let dst_stride_uv = i32::try_from(chroma_stride).ok()?;
            let (dst_y, dst_uv) = dst.split_at_mut(y_size);
            let (dst_u, dst_v) = dst_uv.split_at_mut(uv_size);
            if !i420_copy(
                i420.y_data(),
                i420.stride_y(),
                i420.u_data(),
                i420.stride_u(),
                i420.v_data(),
                i420.stride_v(),
                dst_y,
                dst_stride_y,
                dst_u,
                dst_stride_uv,
                dst_v,
                dst_stride_uv,
                i420.width(),
                i420.height(),
            ) {
                return None;
            }
            Some(yuv_size)
        };

        let value = EncoderCallbackValue {
            rtp_timestamp,
            width: self.width,
            height: self.height,
            dmabuf_done_tx: None,
        };
        let mut guard = self.shared_state.lock().unwrap();
        let Some(encoder) = guard.encoder.as_mut() else {
            return VideoCodecStatus::Uninitialized;
        };
        match encoder.encode(
            EncodeInput::Mmap(&mut fill),
            timestamp_us,
            force_keyframe,
            value,
        ) {
            Ok(()) => VideoCodecStatus::Ok,
            Err(V4l2Error::NoAvailableBuffer) => VideoCodecStatus::NoOutput,
            Err(e) => {
                warn!(target: "v4l2", error = %e, "encode error");
                VideoCodecStatus::Error
            }
        }
    }

    fn register_encode_complete_callback(
        &mut self,
        callback: Option<VideoEncoderEncodedImageCallbackRef<'_>>,
    ) -> VideoCodecStatus {
        let mut guard = self.shared_state.lock().unwrap();
        guard.callback =
            callback.map(|cb| unsafe { VideoEncoderEncodedImageCallbackPtr::from_ref(cb) });
        VideoCodecStatus::Ok
    }

    fn release(&mut self) -> VideoCodecStatus {
        let mut guard = self.shared_state.lock().unwrap();
        guard.encoder = None;
        guard.callback = None;
        info!(target: "v4l2", "V4L2 H.264 encoder released");
        VideoCodecStatus::Ok
    }

    fn set_rates(&mut self, parameters: VideoEncoderRateControlParametersRef<'_>) {
        let bitrate_bps = parameters.target_bitrate_sum_bps();
        self.bitrate_bps = bitrate_bps;
        let mut guard = self.shared_state.lock().unwrap();
        if let Some(ref mut encoder) = guard.encoder
            && let Err(e) = encoder.set_bitrate(bitrate_bps)
        {
            warn!(target: "v4l2", error = %e, "set_bitrate failed");
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
    use shiguredo_webrtc::{
        EnvironmentRef, SdpVideoFormat, SdpVideoFormatRef, VideoDecoder, VideoEncoder,
    };
    use sora_sdk::{CodecDirection, VideoCodecCapability, VideoCodecImplementation};

    use super::V4l2H264Encoder;

    /// V4L2 H.264 ハードウェアエンコーダーの VideoCodecCapability 実装
    pub(crate) struct V4l2VideoCodecCapability {
        pub dmabuf_map: Option<super::DmaBufMap>,
    }

    impl V4l2VideoCodecCapability {
        fn supported_format() -> SdpVideoFormat {
            // Constrained Baseline Profile
            let mut format = SdpVideoFormat::new("H264");
            {
                let mut params = format.parameters_mut();
                params.set("profile-level-id", "42e01f");
                params.set("level-asymmetry-allowed", "1");
                params.set("packetization-mode", "1");
            }
            format
        }
    }

    impl VideoCodecCapability for V4l2VideoCodecCapability {
        fn get_implementation(&self) -> VideoCodecImplementation {
            VideoCodecImplementation::new("v4l2", "V4L2 M2M H.264 Hardware Encoder")
        }

        fn get_supported_formats(&self, direction: CodecDirection) -> Vec<SdpVideoFormat> {
            // エンコードのみ対応
            if direction == CodecDirection::Encoder {
                vec![Self::supported_format()]
            } else {
                Vec::new()
            }
        }

        fn create_video_encoder(
            &self,
            _env: EnvironmentRef<'_>,
            format: SdpVideoFormatRef<'_>,
        ) -> Option<VideoEncoder> {
            let name = format.name().ok()?;
            if name != "H264" {
                return None;
            }
            Some(VideoEncoder::new_with_handler(Box::new(
                V4l2H264Encoder::new(self.dmabuf_map.clone()),
            )))
        }

        fn create_video_decoder(
            &self,
            _env: EnvironmentRef<'_>,
            _format: SdpVideoFormatRef<'_>,
        ) -> Option<VideoDecoder> {
            // V4L2 デコーダーは未対応
            None
        }
    }
}
