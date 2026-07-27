//! libcamera を使用した映像キャプチャ (Raspberry Pi 専用)

use std::collections::HashMap;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use shiguredo_libcamera::{
    CameraManager, ConfigStatus, ControlId, ControlType, Direction, FrameBufferAllocator,
    FrameStatus, PixelFormat, Rectangle, RequestStatus, Size, StreamRole, core, draft, rpi,
};
use shiguredo_webrtc::ffi::*;
use shiguredo_webrtc::{AdaptedVideoTrackSource, I420Buffer, TimestampAligner};
use tracing::{error, info, warn};

use crate::error::BoxError;

// ── DMA-BUF ネイティブバッファ受け渡し ───────────────────────────────────────

/// libcamera → V4L2 エンコーダー間の DMA-BUF 情報
pub(crate) struct DmaBufEntry {
    pub fd: i32,
    pub bytesused: u32,
    pub length: u32,
    /// エンコード完了通知用。送信するとバッファの requeue が許可される。
    pub done_tx: std::sync::mpsc::SyncSender<()>,
}

/// タイムスタンプをキーにした DMA-BUF 受け渡しマップ
pub(crate) type DmaBufMap = Arc<Mutex<HashMap<i64, DmaBufEntry>>>;

/// YU12 (= I420) の FOURCC
const YU12_FOURCC: u32 = u32::from_le_bytes(*b"YU12");

/// libcamera キャプチャのハンドル
///
/// Drop 時にキャプチャスレッドを停止する。
pub(crate) struct LibcameraCapture {
    stop: Arc<AtomicBool>,
    thread: Option<std::thread::JoinHandle<()>>,
}

impl Drop for LibcameraCapture {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Release);
        if let Some(t) = self.thread.take() {
            let _ = t.join();
        }
    }
}

/// libcamera キャプチャを開始する
pub(crate) fn start_libcamera_capture(
    source: AdaptedVideoTrackSource,
    width: u32,
    height: u32,
    controls: Vec<(String, String)>,
    dmabuf_map: Option<DmaBufMap>,
) -> Result<LibcameraCapture, BoxError> {
    let stop = Arc::new(AtomicBool::new(false));
    let stop_clone = stop.clone();

    let thread = std::thread::Builder::new()
        .name("libcamera".to_string())
        .spawn(move || {
            if let Err(e) =
                run_libcamera_loop(source, width, height, stop_clone, &controls, dmabuf_map)
            {
                error!(target: "libcamera", error = %e, "capture error");
            }
        })
        .map_err(|e| format!("libcamera スレッドの起動に失敗: {e}"))?;

    Ok(LibcameraCapture {
        stop,
        thread: Some(thread),
    })
}

/// フレーム情報 (DMA-BUF の fd, offset, length, タイムスタンプ)
type FrameInfo = (i32, u32, u32, u64);

fn run_libcamera_loop(
    source: AdaptedVideoTrackSource,
    width: u32,
    height: u32,
    stop: Arc<AtomicBool>,
    controls: &[(String, String)],
    dmabuf_map: Option<DmaBufMap>,
) -> Result<(), BoxError> {
    let native = dmabuf_map.is_some();
    let shared = Arc::new(Mutex::new((source, TimestampAligner::new())));

    // CameraManager
    let manager = CameraManager::new().map_err(|e| format!("CameraManager の作成に失敗: {e}"))?;
    if manager.cameras_count() == 0 {
        return Err("libcamera: カメラが見つかりません".into());
    }

    // Camera の取得と acquire
    let mut camera = manager
        .get_camera(0)
        .map_err(|e| format!("カメラの取得に失敗: {e}"))?;
    info!(target: "libcamera", camera_id = camera.id(), "camera acquired");
    camera
        .acquire()
        .map_err(|e| format!("カメラの acquire に失敗: {e}"))?;

    // Configuration
    let mut config = camera
        .generate_configuration(&[StreamRole::VideoRecording])
        .map_err(|e| format!("設定の生成に失敗: {e}"))?;

    {
        let mut sc = config
            .at(0)
            .map_err(|e| format!("ストリーム設定の取得に失敗: {e}"))?;
        sc.set_pixel_format(PixelFormat::from_fourcc(YU12_FOURCC));
        sc.set_size(Size { width, height });
    }

    let status = config
        .validate()
        .map_err(|e| format!("設定のバリデーションに失敗: {e}"))?;
    if status == ConfigStatus::Invalid {
        return Err("libcamera: 設定が無効です".into());
    }

    camera
        .configure(&mut config)
        .map_err(|e| format!("カメラの設定に失敗: {e}"))?;

    let (actual_width, actual_height, stride) = {
        let sc = config
            .at(0)
            .map_err(|e| format!("ストリーム設定の取得に失敗: {e}"))?;
        let size = sc.size();
        (size.width, size.height, sc.stride())
    };

    if status == ConfigStatus::Adjusted {
        info!(target: "libcamera", width = actual_width, height = actual_height, "configuration adjusted");
    }

    // Stream と FrameBufferAllocator
    let stream = {
        let sc = config
            .at(0)
            .map_err(|e| format!("ストリーム設定の取得に失敗: {e}"))?;
        sc.stream().ok_or("libcamera: ストリームの取得に失敗")?
    };

    let allocator = FrameBufferAllocator::new(&camera);
    let buffer_count = allocator
        .allocate(&stream)
        .map_err(|e| format!("バッファの割り当てに失敗: {e}"))?;

    // フレーム受信チャネル
    // cookie と Option<FrameInfo> を送信する
    // Startup フレームは None、Success フレームは Some(...)
    let (tx, rx) = std::sync::mpsc::channel::<(u64, Option<FrameInfo>)>();

    let stream_cb = stream.clone();
    camera.on_request_completed(move |completed| {
        if completed.status() != RequestStatus::Complete {
            return;
        }
        if let Some(buffer) = completed.find_buffer(&stream_cb) {
            let meta = buffer.metadata();
            if meta.status == FrameStatus::Success
                && let Some(plane) = buffer.plane(0)
            {
                let timestamp_us = meta.timestamp / 1000;
                let _ = tx.send((
                    completed.cookie(),
                    Some((plane.fd, plane.offset, plane.length, timestamp_us)),
                ));
                return;
            }
            // Startup やエラーフレームは再キューイングのみ
            let _ = tx.send((completed.cookie(), None));
        }
    });

    // コントロール設定のパース
    let parsed_controls = parse_controls(controls);

    // Request の作成
    let mut requests = Vec::with_capacity(buffer_count);
    for i in 0..buffer_count {
        let buffer = allocator
            .get_buffer(&stream, i)
            .map_err(|e| format!("バッファの取得に失敗: {e}"))?;
        let request = camera
            .create_request(i as u64)
            .map_err(|e| format!("リクエストの作成に失敗: {e}"))?;
        request
            .add_buffer(&stream, &buffer)
            .map_err(|e| format!("バッファの追加に失敗: {e}"))?;
        apply_controls(&request, &parsed_controls);
        requests.push(request);
    }

    // カメラ開始
    camera
        .start()
        .map_err(|e| format!("カメラの開始に失敗: {e}"))?;
    for request in &requests {
        camera
            .queue_request(request)
            .map_err(|e| format!("リクエストのキューイングに失敗: {e}"))?;
    }

    info!(
        target: "libcamera",
        width = actual_width,
        height = actual_height,
        stride = stride,
        buffers = buffer_count,
        "capture started"
    );

    let w = actual_width as i32;
    let h = actual_height as i32;

    // メインループ
    loop {
        if stop.load(Ordering::Acquire) {
            break;
        }

        let (cookie, frame_info) = match rx.recv_timeout(Duration::from_millis(100)) {
            Ok(v) => v,
            Err(std::sync::mpsc::RecvTimeoutError::Timeout) => continue,
            Err(std::sync::mpsc::RecvTimeoutError::Disconnected) => break,
        };

        if let Some((fd, offset, length, timestamp_us)) = frame_info {
            if native {
                // ── ネイティブモード: DMA-BUF fd をエンコーダーに直接渡す ──
                let map = dmabuf_map.as_ref().unwrap();
                let (done_tx, done_rx) = std::sync::mpsc::sync_channel(1);
                {
                    let mut guard = map.lock().unwrap();
                    guard.insert(
                        timestamp_us as i64,
                        DmaBufEntry {
                            fd,
                            bytesused: length,
                            length,
                            done_tx,
                        },
                    );
                }

                // 空の I420Buffer で WebRTC パイプラインをトリガー
                if let Ok(mut guard) = shared.lock() {
                    let (ref mut source, ref mut aligner) = *guard;
                    let adapt_result = source.adapt_frame(w, h, timestamp_us as i64);
                    if adapt_result.applied {
                        let ts = aligner
                            .translate(timestamp_us as i64, shiguredo_webrtc::time_millis() * 1000);
                        let dummy = I420Buffer::new(w, h);
                        let video_frame = crate::webrtc_video::video_frame_from_i420(
                            &dummy,
                            ts,
                            (ts * 90 / 1000) as u32,
                        );
                        source.on_frame(&video_frame);
                    } else {
                        // フレームがドロップされた場合はエントリを除去して即 requeue
                        let mut guard = map.lock().unwrap();
                        guard.remove(&(timestamp_us as i64));
                        requeue_request(&camera, &requests, cookie, &parsed_controls);
                        continue;
                    }
                }

                // エンコード完了を待機してから requeue
                let _ = done_rx.recv_timeout(Duration::from_secs(1));
                // タイムアウト時もエントリが残っていれば除去
                {
                    let mut guard = map.lock().unwrap();
                    guard.remove(&(timestamp_us as i64));
                }
                requeue_request(&camera, &requests, cookie, &parsed_controls);
            } else {
                // ── 通常モード: DMA-BUF を mmap してフレームデータを読み取る ──
                //
                // SAFETY: libcamera が割り当てた DMA-BUF を読み取り専用で mmap する。
                // mmap 成功後、バッファの length 分だけ読み取り、直後に munmap する。
                let i420_buf = unsafe {
                    let ptr = libc::mmap(
                        std::ptr::null_mut(),
                        length as usize,
                        libc::PROT_READ,
                        libc::MAP_SHARED,
                        fd,
                        offset as libc::off_t,
                    );
                    if ptr == libc::MAP_FAILED {
                        error!(target: "libcamera", "mmap failed");
                        requeue_request(&camera, &requests, cookie, &parsed_controls);
                        continue;
                    }
                    let slice = std::slice::from_raw_parts(ptr as *const u8, length as usize);
                    let buf = copy_i420_to_buffer(slice, w, h, stride as i32);
                    libc::munmap(ptr, length as usize);
                    buf
                };

                // WebRTC フレームとして供給
                if let Ok(mut guard) = shared.lock() {
                    let (ref mut source, ref mut aligner) = *guard;
                    let adapt_result = source.adapt_frame(w, h, timestamp_us as i64);
                    if adapt_result.applied {
                        let ts = aligner
                            .translate(timestamp_us as i64, shiguredo_webrtc::time_millis() * 1000);
                        let video_frame = if adapt_result.size.adapted_width != w
                            || adapt_result.size.adapted_height != h
                        {
                            let mut scaled = I420Buffer::new(
                                adapt_result.size.adapted_width,
                                adapt_result.size.adapted_height,
                            );
                            scaled.scale_from(&i420_buf);
                            crate::webrtc_video::video_frame_from_i420(
                                &scaled,
                                ts,
                                (ts * 90 / 1000) as u32,
                            )
                        } else {
                            crate::webrtc_video::video_frame_from_i420(
                                &i420_buf,
                                ts,
                                (ts * 90 / 1000) as u32,
                            )
                        };
                        source.on_frame(&video_frame);
                    }
                }

                // 再キューイング
                requeue_request(&camera, &requests, cookie, &parsed_controls);
            }
        }
    }

    // クリーンアップ
    let _ = camera.stop();
    let _ = camera.release();
    info!(target: "libcamera", "capture stopped");

    Ok(())
}

fn requeue_request(
    camera: &shiguredo_libcamera::Camera,
    requests: &[shiguredo_libcamera::Request],
    cookie: u64,
    parsed_controls: &[ParsedControl],
) {
    let idx = cookie as usize;
    requests[idx].reuse();
    apply_controls(&requests[idx], parsed_controls);
    if let Err(e) = camera.queue_request(&requests[idx]) {
        error!(target: "libcamera", error = %e, "queue_request failed");
    }
}

/// I420 (YU12) データを I420Buffer にコピーする (stride を考慮)
///
/// libcamera の YU12 フォーマットはメモリ上に Y, U, V 平面が連続して配置される。
/// stride が width と異なる場合があるため、行ごとにコピーする。
fn copy_i420_to_buffer(data: &[u8], width: i32, height: i32, stride: i32) -> I420Buffer {
    let buf = I420Buffer::new(width, height);

    let w = width as usize;
    let h = height as usize;
    let src_stride = stride as usize;
    let src_stride_uv = src_stride / 2;

    // SAFETY: I420Buffer::new が割り当てた参照カウント付きバッファを排他的に所有している。
    // `buf` が生きている間、派生したポインタは有効である。
    unsafe {
        let refcounted = buf.as_refcounted_ptr();
        let raw = webrtc_I420Buffer_refcounted_get(refcounted);

        let dst_y = webrtc_I420Buffer_MutableDataY(raw);
        let dst_u = webrtc_I420Buffer_MutableDataU(raw);
        let dst_v = webrtc_I420Buffer_MutableDataV(raw);
        let dst_stride_y = webrtc_I420Buffer_StrideY(raw) as usize;
        let dst_stride_u = webrtc_I420Buffer_StrideU(raw) as usize;
        let dst_stride_v = webrtc_I420Buffer_StrideV(raw) as usize;

        // Y plane
        for row in 0..h {
            std::ptr::copy_nonoverlapping(
                data.as_ptr().add(row * src_stride),
                dst_y.add(row * dst_stride_y),
                w,
            );
        }

        // U plane (Y plane の直後)
        let u_offset = src_stride * h;
        for row in 0..h / 2 {
            std::ptr::copy_nonoverlapping(
                data.as_ptr().add(u_offset + row * src_stride_uv),
                dst_u.add(row * dst_stride_u),
                w / 2,
            );
        }

        // V plane (U plane の直後)
        let v_offset = u_offset + src_stride_uv * (h / 2);
        for row in 0..h / 2 {
            std::ptr::copy_nonoverlapping(
                data.as_ptr().add(v_offset + row * src_stride_uv),
                dst_v.add(row * dst_stride_v),
                w / 2,
            );
        }
    }

    buf
}

// ── コントロール設定 ─────────────────────────────────────────────────────────

/// パース済みコントロール値
enum ControlValue {
    Bool(bool),
    I32(i32),
    I64(i64),
    F32(f32),
    I32Array(Vec<i32>),
    I64Array(Vec<i64>),
    F32Array(Vec<f32>),
    Rect(Rectangle),
    RectArray(Vec<Rectangle>),
}

/// パース済みコントロール (ControlId + 値)
struct ParsedControl {
    id: &'static ControlId,
    value: ControlValue,
}

/// 全コントロール ID のテーブル (名前引き用)
fn all_control_ids() -> &'static [&'static ControlId] {
    static IDS: &[&ControlId] = &[
        // core
        &core::AE_ENABLE,
        &core::AE_STATE,
        &core::AE_METERING_MODE,
        &core::AE_CONSTRAINT_MODE,
        &core::AE_EXPOSURE_MODE,
        &core::EXPOSURE_VALUE,
        &core::EXPOSURE_TIME,
        &core::EXPOSURE_TIME_MODE,
        &core::ANALOGUE_GAIN,
        &core::ANALOGUE_GAIN_MODE,
        &core::AE_FLICKER_MODE,
        &core::AE_FLICKER_PERIOD,
        &core::AE_FLICKER_DETECTED,
        &core::BRIGHTNESS,
        &core::CONTRAST,
        &core::LUX,
        &core::AWB_ENABLE,
        &core::AWB_MODE,
        &core::AWB_LOCKED,
        &core::COLOUR_GAINS,
        &core::COLOUR_TEMPERATURE,
        &core::SATURATION,
        &core::SENSOR_BLACK_LEVELS,
        &core::SHARPNESS,
        &core::FOCUS_FOM,
        &core::COLOUR_CORRECTION_MATRIX,
        &core::SCALER_CROP,
        &core::DIGITAL_GAIN,
        &core::FRAME_DURATION,
        &core::FRAME_DURATION_LIMITS,
        &core::SENSOR_TEMPERATURE,
        &core::SENSOR_TIMESTAMP,
        &core::AF_MODE,
        &core::AF_RANGE,
        &core::AF_SPEED,
        &core::AF_METERING,
        &core::AF_WINDOWS,
        &core::AF_TRIGGER,
        &core::AF_PAUSE,
        &core::LENS_POSITION,
        &core::AF_STATE,
        &core::AF_PAUSE_STATE,
        &core::HDR_MODE,
        &core::HDR_CHANNEL,
        &core::GAMMA,
        &core::DEBUG_METADATA_ENABLE,
        &core::FRAME_WALL_CLOCK,
        &core::WDR_MODE,
        &core::WDR_STRENGTH,
        &core::WDR_MAX_BRIGHT_PIXELS,
        &core::LENS_DEWARP_ENABLE,
        &core::LENS_SHADING_CORRECTION_ENABLE,
        // draft
        &draft::AE_PRECAPTURE_TRIGGER,
        &draft::NOISE_REDUCTION_MODE,
        &draft::COLOR_CORRECTION_ABERRATION_MODE,
        &draft::AWB_STATE,
        &draft::SENSOR_ROLLING_SHUTTER_SKEW,
        &draft::LENS_SHADING_MAP_MODE,
        &draft::PIPELINE_DEPTH,
        &draft::MAX_LATENCY,
        &draft::TEST_PATTERN_MODE,
        &draft::FACE_DETECT_MODE,
        &draft::FACE_DETECT_FACE_RECTANGLES,
        &draft::FACE_DETECT_FACE_SCORES,
        &draft::FACE_DETECT_FACE_LANDMARKS,
        &draft::FACE_DETECT_FACE_IDS,
        // rpi
        &rpi::STATS_OUTPUT_ENABLE,
        &rpi::BCM2835_STATS_OUTPUT,
        &rpi::SCALER_CROPS,
        &rpi::PISP_STATS_OUTPUT,
        &rpi::SYNC_MODE,
        &rpi::SYNC_READY,
        &rpi::SYNC_TIMER,
        &rpi::SYNC_FRAMES,
        &rpi::CNN_OUTPUT_TENSOR,
        &rpi::CNN_OUTPUT_TENSOR_INFO,
        &rpi::CNN_ENABLE_INPUT_TENSOR,
        &rpi::CNN_INPUT_TENSOR,
        &rpi::CNN_INPUT_TENSOR_INFO,
        &rpi::CNN_KPI_INFO,
    ];
    IDS
}

/// コントロール名から ControlId を検索する
fn find_control_id(name: &str) -> Option<&'static ControlId> {
    all_control_ids()
        .iter()
        .find(|id| id.name() == name)
        .copied()
}

/// enum 文字列を i32 値に変換する
///
/// 主要な enum コントロールについて文字列名での指定をサポートする。
/// 数値がそのまま指定された場合はそれを返す。
fn resolve_enum_value(control_name: &str, value_str: &str) -> Option<i32> {
    // 数値の場合はそのままパース
    if let Ok(v) = value_str.parse::<i32>() {
        return Some(v);
    }

    match control_name {
        "AfMode" => match value_str {
            "Manual" => Some(core::af_mode::MANUAL),
            "Auto" => Some(core::af_mode::AUTO),
            "Continuous" => Some(core::af_mode::CONTINUOUS),
            _ => None,
        },
        "AfRange" => match value_str {
            "Normal" => Some(core::af_range::NORMAL),
            "Macro" => Some(core::af_range::MACRO),
            "Full" => Some(core::af_range::FULL),
            _ => None,
        },
        "AfSpeed" => match value_str {
            "Normal" => Some(core::af_speed::NORMAL),
            "Fast" => Some(core::af_speed::FAST),
            _ => None,
        },
        "AfTrigger" => match value_str {
            "Start" => Some(core::af_trigger::START),
            "Cancel" => Some(core::af_trigger::CANCEL),
            _ => None,
        },
        "AeMeteringMode" => match value_str {
            "CentreWeighted" => Some(core::ae_metering_mode::CENTRE_WEIGHTED),
            "Spot" => Some(core::ae_metering_mode::SPOT),
            "Matrix" => Some(core::ae_metering_mode::MATRIX),
            "Custom" => Some(core::ae_metering_mode::CUSTOM),
            _ => None,
        },
        "AeConstraintMode" => match value_str {
            "Normal" => Some(core::ae_constraint_mode::NORMAL),
            "Highlight" => Some(core::ae_constraint_mode::HIGHLIGHT),
            "Shadows" => Some(core::ae_constraint_mode::SHADOWS),
            "Custom" => Some(core::ae_constraint_mode::CUSTOM),
            _ => None,
        },
        "AeExposureMode" => match value_str {
            "Normal" => Some(core::ae_exposure_mode::NORMAL),
            "Short" => Some(core::ae_exposure_mode::SHORT),
            "Long" => Some(core::ae_exposure_mode::LONG),
            "Custom" => Some(core::ae_exposure_mode::CUSTOM),
            _ => None,
        },
        "ExposureTimeMode" => match value_str {
            "Auto" => Some(core::exposure_time_mode::AUTO),
            "Manual" => Some(core::exposure_time_mode::MANUAL),
            _ => None,
        },
        "AnalogueGainMode" => match value_str {
            "Auto" => Some(core::analogue_gain_mode::AUTO),
            "Manual" => Some(core::analogue_gain_mode::MANUAL),
            _ => None,
        },
        "AwbMode" => match value_str {
            "Auto" => Some(core::awb_mode::AUTO),
            "Incandescent" => Some(core::awb_mode::INCANDESCENT),
            "Tungsten" => Some(core::awb_mode::TUNGSTEN),
            "Fluorescent" => Some(core::awb_mode::FLUORESCENT),
            "Indoor" => Some(core::awb_mode::INDOOR),
            "Daylight" => Some(core::awb_mode::DAYLIGHT),
            "Cloudy" => Some(core::awb_mode::CLOUDY),
            "Custom" => Some(core::awb_mode::CUSTOM),
            _ => None,
        },
        "HdrMode" => match value_str {
            "Off" => Some(core::hdr_mode::OFF),
            "MultiExposureUnmerged" => Some(core::hdr_mode::MULTI_EXPOSURE_UNMERGED),
            "MultiExposure" => Some(core::hdr_mode::MULTI_EXPOSURE),
            "SingleExposure" => Some(core::hdr_mode::SINGLE_EXPOSURE),
            "Night" => Some(core::hdr_mode::NIGHT),
            _ => None,
        },
        "NoiseReductionMode" => match value_str {
            "Off" => Some(draft::noise_reduction_mode::OFF),
            "Fast" => Some(draft::noise_reduction_mode::FAST),
            "HighQuality" => Some(draft::noise_reduction_mode::HIGH_QUALITY),
            "Minimal" => Some(draft::noise_reduction_mode::MINIMAL),
            "ZSL" => Some(draft::noise_reduction_mode::ZSL),
            _ => None,
        },
        _ => None,
    }
}

/// Rectangle 文字列 "x,y,width,height" をパースする
fn parse_rectangle(s: &str) -> Option<Rectangle> {
    let parts: Vec<&str> = s.split(',').collect();
    if parts.len() != 4 {
        return None;
    }
    Some(Rectangle {
        x: parts[0].trim().parse().ok()?,
        y: parts[1].trim().parse().ok()?,
        width: parts[2].trim().parse().ok()?,
        height: parts[3].trim().parse().ok()?,
    })
}

/// コントロール値文字列を ControlType に基づいてパースする
fn parse_control_value(id: &ControlId, value_str: &str) -> Option<ControlValue> {
    match id.control_type() {
        ControlType::Bool => {
            let v = match value_str {
                "0" | "false" => false,
                "1" | "true" => true,
                _ => return None,
            };
            Some(ControlValue::Bool(v))
        }
        ControlType::Int32 => {
            // enum 文字列の解決を試みる
            if let Some(v) = resolve_enum_value(id.name(), value_str) {
                return Some(ControlValue::I32(v));
            }
            // カンマ区切りの配列
            if value_str.contains(',') {
                let values: Option<Vec<i32>> = value_str
                    .split(',')
                    .map(|s| s.trim().parse().ok())
                    .collect();
                return values.map(ControlValue::I32Array);
            }
            value_str.parse().ok().map(ControlValue::I32)
        }
        ControlType::Int64 => {
            // カンマ区切りの配列
            if value_str.contains(',') {
                let values: Option<Vec<i64>> = value_str
                    .split(',')
                    .map(|s| s.trim().parse().ok())
                    .collect();
                return values.map(ControlValue::I64Array);
            }
            value_str.parse().ok().map(ControlValue::I64)
        }
        ControlType::Float => {
            // カンマ区切りの配列
            if value_str.contains(',') {
                let values: Option<Vec<f32>> = value_str
                    .split(',')
                    .map(|s| s.trim().parse().ok())
                    .collect();
                return values.map(ControlValue::F32Array);
            }
            value_str.parse().ok().map(ControlValue::F32)
        }
        ControlType::Rectangle => {
            // セミコロン区切りで複数矩形
            if value_str.contains(';') {
                let rects: Option<Vec<Rectangle>> = value_str
                    .split(';')
                    .map(|s| parse_rectangle(s.trim()))
                    .collect();
                return rects.map(ControlValue::RectArray);
            }
            parse_rectangle(value_str).map(ControlValue::Rect)
        }
        _ => {
            warn!(
                target: "libcamera",
                control = id.name(),
                control_type = ?id.control_type(),
                "unsupported control type"
            );
            None
        }
    }
}

/// CLI から渡されたコントロール文字列をパースする
fn parse_controls(controls: &[(String, String)]) -> Vec<ParsedControl> {
    let mut parsed = Vec::with_capacity(controls.len());
    for (key, value_str) in controls {
        let Some(id) = find_control_id(key) else {
            warn!(target: "libcamera", control = %key, "unknown control");
            continue;
        };

        // Out 方向のコントロールは設定不可
        if id.direction() == Direction::Out {
            warn!(target: "libcamera", control = %key, "read-only control, skipping");
            continue;
        }

        match parse_control_value(id, value_str) {
            Some(value) => {
                info!(target: "libcamera", control = key.as_str(), value = value_str.as_str(), "control set");
                parsed.push(ParsedControl { id, value });
            }
            None => {
                warn!(
                    target: "libcamera",
                    control = %key,
                    value = %value_str,
                    "invalid control value"
                );
            }
        }
    }
    parsed
}

/// パース済みコントロールを Request に適用する
fn apply_controls(request: &shiguredo_libcamera::Request, controls: &[ParsedControl]) {
    if controls.is_empty() {
        return;
    }
    let mut cl = request.controls();
    for ctrl in controls {
        match &ctrl.value {
            ControlValue::Bool(v) => cl.set_bool(ctrl.id, *v),
            ControlValue::I32(v) => cl.set_i32(ctrl.id, *v),
            ControlValue::I64(v) => cl.set_i64(ctrl.id, *v),
            ControlValue::F32(v) => cl.set_f32(ctrl.id, *v),
            ControlValue::I32Array(v) => cl.set_i32_array(ctrl.id, v),
            ControlValue::I64Array(v) => cl.set_i64_array(ctrl.id, v),
            ControlValue::F32Array(v) => cl.set_f32_array(ctrl.id, v),
            ControlValue::Rect(v) => cl.set_rectangle(ctrl.id, *v),
            ControlValue::RectArray(v) => cl.set_rectangle_array(ctrl.id, v),
        }
    }
}
