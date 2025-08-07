#include "rtc/fake_audio_capturer.h"

#if defined(USE_FAKE_CAPTURE_DEVICE)

#include <cmath>
#include <chrono>

// WebRTC
#include <rtc_base/logging.h>

FakeAudioCapturer::FakeAudioCapturer(Config config)
    : config_(config), env_(webrtc::CreateEnvironment()) {}

FakeAudioCapturer::~FakeAudioCapturer() {
  Terminate();
}

void FakeAudioCapturer::TriggerBeep() {
  std::lock_guard<std::mutex> lock(beep_mutex_);
  trigger_beep_ = true;
}

int32_t FakeAudioCapturer::ActiveAudioLayer(AudioLayer* audioLayer) const {
  *audioLayer = AudioDeviceModule::kDummyAudio;
  return 0;
}

int32_t FakeAudioCapturer::RegisterAudioCallback(
    webrtc::AudioTransport* audioCallback) {
  if (device_buffer_) {
    device_buffer_->RegisterAudioCallback(audioCallback);
  }
  return 0;
}

int32_t FakeAudioCapturer::Init() {
  device_buffer_ = std::make_unique<webrtc::AudioDeviceBuffer>(&env_.task_queue_factory());
  initialized_ = true;
  return 0;
}

int32_t FakeAudioCapturer::Terminate() {
  initialized_ = false;
  is_recording_ = false;
  microphone_initialized_ = false;
  recording_initialized_ = false;
  
  if (audio_thread_ && audio_thread_->joinable()) {
    stop_audio_thread_ = true;
    audio_thread_->join();
    audio_thread_.reset();
  }
  
  device_buffer_.reset();
  return 0;
}

bool FakeAudioCapturer::Initialized() const {
  return initialized_;
}

int32_t FakeAudioCapturer::PlayoutIsAvailable(bool* available) {
  *available = false;
  return 0;
}

int32_t FakeAudioCapturer::RecordingIsAvailable(bool* available) {
  *available = true;
  return 0;
}

int32_t FakeAudioCapturer::InitRecording() {
  recording_initialized_ = true;
  device_buffer_->SetRecordingSampleRate(config_.sample_rate);
  device_buffer_->SetRecordingChannels(config_.channels);
  return 0;
}

bool FakeAudioCapturer::RecordingIsInitialized() const {
  return recording_initialized_;
}

int32_t FakeAudioCapturer::StartRecording() {
  if (!audio_thread_) {
    stop_audio_thread_ = false;
    audio_thread_ = std::make_unique<std::thread>([this] { AudioThread(); });
  }
  is_recording_ = true;
  return 0;
}

int32_t FakeAudioCapturer::StopRecording() {
  is_recording_ = false;
  if (audio_thread_ && audio_thread_->joinable()) {
    stop_audio_thread_ = true;
    audio_thread_->join();
    audio_thread_.reset();
  }
  return 0;
}

bool FakeAudioCapturer::Recording() const {
  return is_recording_;
}

int32_t FakeAudioCapturer::InitMicrophone() {
  microphone_initialized_ = true;
  return 0;
}

bool FakeAudioCapturer::MicrophoneIsInitialized() const {
  return microphone_initialized_;
}

void FakeAudioCapturer::AudioThread() {
  // 10ms ごとにオーディオデータを生成
  const int samples_per_10ms = config_.sample_rate / 100;
  std::vector<int16_t> buffer(samples_per_10ms * config_.channels);
  
  while (!stop_audio_thread_) {
    auto start = std::chrono::steady_clock::now();
    
    // ビープ音の生成またはサイレンス
    {
      std::lock_guard<std::mutex> lock(beep_mutex_);
      if (trigger_beep_) {
        trigger_beep_ = false;
        beep_samples_remaining_ = (beep_duration_ms_ * config_.sample_rate) / 1000;
      }
    }
    
    if (beep_samples_remaining_ > 0) {
      GenerateBeep(buffer, samples_per_10ms);
      beep_samples_remaining_ -= samples_per_10ms;
      if (beep_samples_remaining_ < 0) {
        beep_samples_remaining_ = 0;
      }
    } else {
      GenerateSilence(buffer, samples_per_10ms);
    }
    
    // WebRTC にオーディオデータを送信
    if (device_buffer_ && is_recording_) {
      device_buffer_->SetRecordedBuffer(buffer.data(), samples_per_10ms);
      device_buffer_->DeliverRecordedData();
    }
    
    // 10ms スリープ
    auto end = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    if (elapsed < std::chrono::milliseconds(10)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10) - elapsed);
    }
  }
}

void FakeAudioCapturer::GenerateBeep(std::vector<int16_t>& buffer, int samples) {
  const double frequency = beep_frequency_;
  const double amplitude = 16000;  // 音量（最大32767の半分程度）
  const double sample_rate = config_.sample_rate;
  
  for (int i = 0; i < samples; ++i) {
    double t = i / sample_rate;
    int16_t value = static_cast<int16_t>(amplitude * sin(2.0 * M_PI * frequency * t));
    
    for (int ch = 0; ch < config_.channels; ++ch) {
      buffer[i * config_.channels + ch] = value;
    }
  }
}

void FakeAudioCapturer::GenerateSilence(std::vector<int16_t>& buffer, int samples) {
  std::fill(buffer.begin(), buffer.end(), 0);
}

#endif  // USE_FAKE_CAPTURE_DEVICE