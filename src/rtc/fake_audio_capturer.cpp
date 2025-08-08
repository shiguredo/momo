#include "rtc/fake_audio_capturer.h"

#if defined(USE_FAKE_CAPTURE_DEVICE)

#include <chrono>
#include <cmath>

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
  device_buffer_ =
      std::make_unique<webrtc::AudioDeviceBuffer>(&env_.task_queue_factory());
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

  auto next_time = std::chrono::steady_clock::now();
  const auto interval = std::chrono::microseconds(10000);  // 10ms = 10000us

  while (!stop_audio_thread_) {
    // ビープ音の生成またはサイレンス
    {
      std::lock_guard<std::mutex> lock(beep_mutex_);
      if (trigger_beep_) {
        trigger_beep_ = false;
        beep_samples_remaining_ =
            (beep_duration_ms_ * config_.sample_rate) / 1000;
        beep_phase_ = 0.0;  // 位相をリセット
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

    // 正確な 10ms 間隔を維持
    next_time += interval;
    std::this_thread::sleep_until(next_time);
  }
}

void FakeAudioCapturer::GenerateBeep(std::vector<int16_t>& buffer,
                                     int samples) {
  const double frequency = beep_frequency_;
  const double amplitude = 16000;  // 音量（最大32767の半分程度）
  const double sample_rate = config_.sample_rate;
  const double phase_increment = 2.0 * M_PI * frequency / sample_rate;

  for (int i = 0; i < samples; ++i) {
    int16_t value = static_cast<int16_t>(amplitude * sin(beep_phase_));
    beep_phase_ += phase_increment;

    // 位相を 0 ~ 2π の範囲に保つ
    if (beep_phase_ >= 2.0 * M_PI) {
      beep_phase_ -= 2.0 * M_PI;
    }

    for (int ch = 0; ch < config_.channels; ++ch) {
      buffer[i * config_.channels + ch] = value;
    }
  }
}

void FakeAudioCapturer::GenerateSilence(std::vector<int16_t>& buffer,
                                        int samples) {
  std::fill(buffer.begin(), buffer.end(), 0);
}

#endif  // USE_FAKE_CAPTURE_DEVICE