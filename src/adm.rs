//! AudioDeviceModule 状態管理
//!
//! p2p モードと ayame モードで共通の ADM 状態と AudioDeviceModuleHandler 構築ロジック。

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};

use shiguredo_audio_device::{AudioFormat, AudioFrame};
use shiguredo_webrtc::{AudioDeviceModuleHandler, AudioTransportRef};

// ─── ADM 状態 ─────────────────────────────────────────────────────────────────

/// AudioDeviceModule の録音状態を保持する共有状態
#[derive(Clone)]
pub struct AdmState {
    recording: Arc<AtomicBool>,
    audio_transport: Arc<Mutex<Option<AudioTransportRef>>>,
}

impl AdmState {
    pub fn new() -> Self {
        Self {
            recording: Arc::new(AtomicBool::new(false)),
            audio_transport: Arc::new(Mutex::new(None)),
        }
    }

    /// 録音データを WebRTC の AudioTransport に転送する
    pub fn on_recorded_data(
        &self,
        audio_data: *const u8,
        n_samples: usize,
        n_bytes_per_sample: usize,
        n_channels: usize,
        samples_per_sec: u32,
    ) {
        if !self.recording.load(Ordering::SeqCst) {
            return;
        }
        let transport = { *self.audio_transport.lock().unwrap() };
        let Some(transport) = transport else { return };
        let mut new_mic_level = 0u32;
        let _ = unsafe {
            transport.recorded_data_is_available(
                audio_data,
                n_samples,
                n_bytes_per_sample,
                n_channels,
                samples_per_sec,
                0,
                0,
                0,
                false,
                &mut new_mic_level,
                None,
            )
        };
    }

    /// AudioCapture コールバックから受け取ったフレームを WebRTC に転送する
    ///
    /// S16 はそのまま転送し、F32 は S16 に変換してから転送する。
    pub fn on_audio_frame(&self, frame: &AudioFrame<'_>) {
        let n_channels = frame.channels as usize;
        let samples_per_sec = frame.sample_rate as u32;
        match frame.format {
            AudioFormat::S16 => {
                self.on_recorded_data(
                    frame.data.as_ptr(),
                    frame.frames as usize,
                    2 * n_channels,
                    n_channels,
                    samples_per_sec,
                );
            }
            AudioFormat::F32 => {
                if let Some(f32_data) = frame.as_f32() {
                    let s16: Vec<i16> = f32_data
                        .iter()
                        .map(|&s| (s.clamp(-1.0, 1.0) * i16::MAX as f32) as i16)
                        .collect();
                    self.on_recorded_data(
                        s16.as_ptr() as *const u8,
                        frame.frames as usize,
                        2 * n_channels,
                        n_channels,
                        samples_per_sec,
                    );
                }
            }
        }
    }
}

// ─── AudioDeviceModuleHandler 構築 ──────────────────────────────────────────

/// 外部音声キャプチャ用の AudioDeviceModuleHandler 実装
struct ExternalAdmHandler {
    audio_transport: Arc<Mutex<Option<AudioTransportRef>>>,
    recording: Arc<AtomicBool>,
}

// SAFETY: AudioTransportRef は WebRTC 内部で排他制御されているため
// ExternalAdmHandler をスレッド間で共有しても安全である。
unsafe impl Sync for ExternalAdmHandler {}

impl AudioDeviceModuleHandler for ExternalAdmHandler {
    fn register_audio_callback(&self, audio_transport: Option<AudioTransportRef>) -> i32 {
        *self.audio_transport.lock().unwrap() = audio_transport;
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
            Some((
                "External Recording".to_string(),
                "external-recording".to_string(),
            ))
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

pub fn build_adm_handler(state: &AdmState) -> Box<dyn AudioDeviceModuleHandler> {
    Box::new(ExternalAdmHandler {
        audio_transport: Arc::clone(&state.audio_transport),
        recording: Arc::clone(&state.recording),
    })
}
