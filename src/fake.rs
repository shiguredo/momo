use std::f64::consts::PI;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Duration;

use shiguredo_webrtc::{
    AdaptedVideoTrackSource, AudioDeviceModule, AudioDeviceModuleHandler, AudioTransportRef,
    I420Buffer,
};

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
pub(crate) fn create_dummy_adm(
    env: &shiguredo_webrtc::Environment,
) -> Result<AudioDeviceModule, crate::error::BoxError> {
    AudioDeviceModule::new(env, shiguredo_webrtc::AudioDeviceModuleAudioLayer::Dummy)
        .map_err(crate::error::wrtc_err)
}

// ─── フェイク音声 ─────────────────────────────────────────────────────────────

/// ビープ音の周波数 (Hz)
const BEEP_FREQUENCY: f64 = 1000.0;
/// ビープ音の長さ (ミリ秒)
const BEEP_DURATION_MS: u32 = 100;
/// ビープ音の振幅 (最大 32767 の約半分)
const BEEP_AMPLITUDE: f64 = 16000.0;
/// サンプルレート (Hz)
const SAMPLE_RATE: u32 = 48000;
/// チャンネル数
const CHANNELS: usize = 1;

/// フェイク音声のビープトリガー
///
/// 映像スレッドから色相サイクル一周時に `trigger()` を呼び出す。
/// 音声スレッドが `take()` でトリガーを消費してビープ音を生成する。
#[derive(Clone)]
pub(crate) struct BeepTrigger {
    flag: Arc<AtomicBool>,
}

impl BeepTrigger {
    pub(crate) fn new() -> Self {
        Self {
            flag: Arc::new(AtomicBool::new(false)),
        }
    }

    /// ビープ音をトリガーする (映像スレッドから呼ぶ)
    pub(crate) fn trigger(&self) {
        self.flag.store(true, Ordering::Release);
    }

    /// トリガーを消費する (音声スレッドから呼ぶ)
    fn take(&self) -> bool {
        self.flag.swap(false, Ordering::AcqRel)
    }
}

/// フェイク音声キャプチャの内部状態
#[derive(Clone)]
struct FakeAudioState {
    recording: Arc<AtomicBool>,
    audio_transport: Arc<std::sync::Mutex<Option<AudioTransportRef>>>,
    beep_trigger: BeepTrigger,
    stop: Arc<AtomicBool>,
}

/// フェイク音声キャプチャ
///
/// カスタム AudioDeviceModule を使って 10ms ごとに PCM データを WebRTC に送信する。
/// 通常は無音を送信し、`BeepTrigger::trigger()` が呼ばれると
/// 1000Hz のビープ音を 100ms 間生成する。
pub(crate) struct FakeAudioCapturer {
    adm: AudioDeviceModule,
    state: FakeAudioState,
    handle: Option<std::thread::JoinHandle<()>>,
}

struct FakeAudioHandler {
    recording: Arc<AtomicBool>,
    audio_transport: Arc<std::sync::Mutex<Option<AudioTransportRef>>>,
}

impl AudioDeviceModuleHandler for FakeAudioHandler {
    fn register_audio_callback(&self, transport: Option<AudioTransportRef>) -> i32 {
        let mut stored = self.audio_transport.lock().unwrap();
        *stored = transport;
        0
    }

    fn init(&self) -> i32 {
        0
    }

    fn terminate(&self) -> i32 {
        0
    }

    fn initialized(&self) -> bool {
        true
    }

    fn recording_devices(&self) -> i16 {
        1
    }

    fn recording_device_name(&self, index: u16) -> Option<(String, String)> {
        if index == 0 {
            Some(("Fake Recording".to_string(), "fake-recording".to_string()))
        } else {
            None
        }
    }

    fn recording_is_available(&self, available: &mut bool) -> i32 {
        *available = true;
        0
    }

    fn init_recording(&self) -> i32 {
        0
    }

    fn recording_is_initialized(&self) -> bool {
        true
    }

    fn start_recording(&self) -> i32 {
        self.recording.store(true, Ordering::SeqCst);
        0
    }

    fn stop_recording(&self) -> i32 {
        self.recording.store(false, Ordering::SeqCst);
        0
    }

    fn recording(&self) -> bool {
        self.recording.load(Ordering::SeqCst)
    }
}

impl FakeAudioCapturer {
    pub(crate) fn new(beep_trigger: BeepTrigger) -> Self {
        let recording = Arc::new(AtomicBool::new(false));
        let audio_transport = Arc::new(std::sync::Mutex::new(None));
        let stop = Arc::new(AtomicBool::new(false));

        let adm = AudioDeviceModule::new_with_handler(Box::new(FakeAudioHandler {
            recording: Arc::clone(&recording),
            audio_transport: Arc::clone(&audio_transport),
        }));

        let state = FakeAudioState {
            recording,
            audio_transport,
            beep_trigger,
            stop,
        };

        Self {
            adm,
            state,
            handle: None,
        }
    }

    pub(crate) fn audio_device_module(&self) -> AudioDeviceModule {
        self.adm.clone()
    }

    pub(crate) fn start(&mut self) {
        if self.handle.is_some() {
            return;
        }

        let state = self.state.clone();
        let handle = std::thread::Builder::new()
            .name("fake-audio".to_string())
            .spawn(move || {
                audio_thread(state);
            })
            .expect("fake audio thread spawn failed");

        self.handle = Some(handle);
    }
}

impl Drop for FakeAudioCapturer {
    fn drop(&mut self) {
        self.state.stop.store(true, Ordering::Release);
        if let Some(handle) = self.handle.take() {
            let _ = handle.join();
        }
    }
}

/// 10ms ごとに PCM データを生成して WebRTC に送信するスレッド
fn audio_thread(state: FakeAudioState) {
    let samples_per_10ms = (SAMPLE_RATE / 100) as usize;
    let mut buffer = vec![0i16; samples_per_10ms * CHANNELS];

    let mut beep_samples_remaining: i32 = 0;
    let mut beep_phase: f64 = 0.0;
    let phase_increment = 2.0 * PI * BEEP_FREQUENCY / SAMPLE_RATE as f64;

    let interval = Duration::from_millis(10);
    let mut next_time = std::time::Instant::now();

    while !state.stop.load(Ordering::Acquire) {
        // ビープトリガーをチェック
        if state.beep_trigger.take() {
            beep_samples_remaining = (BEEP_DURATION_MS * SAMPLE_RATE / 1000) as i32;
            beep_phase = 0.0;
        }

        // ビープ音またはサイレンスを生成
        if beep_samples_remaining > 0 {
            for sample in buffer.iter_mut() {
                *sample = (BEEP_AMPLITUDE * beep_phase.sin()) as i16;
                beep_phase += phase_increment;
                if beep_phase >= 2.0 * PI {
                    beep_phase -= 2.0 * PI;
                }
            }
            beep_samples_remaining -= samples_per_10ms as i32;
            if beep_samples_remaining < 0 {
                beep_samples_remaining = 0;
            }
        } else {
            buffer.fill(0);
        }

        // WebRTC に送信
        if state.recording.load(Ordering::SeqCst) {
            let transport = {
                let stored = state.audio_transport.lock().unwrap();
                *stored
            };
            if let Some(transport) = transport {
                let mut new_mic_level = 0;
                // SAFETY: buffer は有効な i16 スライスであり、WebRTC の AudioTransport に
                // 10ms 分の PCM データを渡すだけなのでメモリ安全。
                let _ = unsafe {
                    transport.recorded_data_is_available(
                        buffer.as_ptr() as *const u8,
                        samples_per_10ms,
                        2 * CHANNELS,
                        CHANNELS,
                        SAMPLE_RATE,
                        0,
                        0,
                        0,
                        false,
                        &mut new_mic_level,
                        None,
                    )
                };
            }
        }

        // 10ms 間隔を維持
        next_time += interval;
        let now = std::time::Instant::now();
        if next_time > now {
            std::thread::sleep(next_time - now);
        }
    }
}

// ─── Send ラッパー ────────────────────────────────────────────────────────────

/// AudioDeviceModule を Send にするためのラッパー。
///
/// WebRTC の AudioDeviceModule は内部的にスレッドセーフな参照カウントオブジェクトだが、
/// NonNull を含むため自動で Send を実装しない。tokio::spawn を跨ぐために必要。
pub(crate) struct SendableAdm(pub(crate) AudioDeviceModule);

// SAFETY: AudioDeviceModule は C++ の参照カウント付きオブジェクトであり、
// スレッド間で安全に移動できる。
unsafe impl Send for SendableAdm {}

// ─── フェイク映像スレッド ─────────────────────────────────────────────────────

/// raden でアニメーションフレームを生成し AdaptedVideoTrackSource に供給するスレッドを起動する
///
/// `beep_trigger` が Some の場合、色相サイクル一周ごとにビープ音をトリガーする。
pub(crate) fn start_fake_video_thread(
    source: AdaptedVideoTrackSource,
    width: i32,
    height: i32,
    fps: u32,
    beep_trigger: Option<BeepTrigger>,
    #[cfg(feature = "player")] preview_tx: Option<
        std::sync::mpsc::SyncSender<crate::preview::PreviewFrame>,
    >,
) {
    std::thread::Builder::new()
        .name("fake-video".to_string())
        .spawn(move || {
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

                // 色相サイクル一周でビープ音をトリガー
                if frame_idx > 0
                    && frame_idx.is_multiple_of(360)
                    && let Some(ref trigger) = beep_trigger
                {
                    trigger.trigger();
                }

                // BGRA → I420 変換してフレームを供給
                let timestamp_us = shiguredo_webrtc::time_millis() * 1000;
                let adapt_result = source.adapt_frame(width, height, timestamp_us);
                if adapt_result.applied {
                    let i420 = bgra_to_i420(image.data(), width, height);

                    // プレビューウィンドウへフレームを送信 (ベストエフォート)
                    #[cfg(feature = "player")]
                    if let Some(ref tx) = preview_tx {
                        let preview_frame = crate::preview::extract_preview_frame(
                            &i420,
                            width,
                            height,
                            timestamp_us,
                        );
                        let _ = tx.try_send(preview_frame);
                    }

                    let frame = crate::webrtc_video::video_frame_from_i420(
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
        .expect("fake video thread spawn failed");
}
