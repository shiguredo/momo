#ifndef RTC_FAKE_AUDIO_CAPTURER_H_
#define RTC_FAKE_AUDIO_CAPTURER_H_

#if defined(USE_FAKE_CAPTURE_DEVICE)

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

// WebRTC
#include <api/environment/environment_factory.h>
#include <api/make_ref_counted.h>
#include <modules/audio_device/audio_device_buffer.h>
#include <modules/audio_device/include/audio_device.h>
#include <rtc_base/ref_counted_object.h>

class FakeAudioCapturer : public webrtc::AudioDeviceModule {
 public:
  struct Config {
    int sample_rate = 48000;
    int channels = 1;
    int fps = 30;  // ビデオのFPSと同期
  };

  static webrtc::scoped_refptr<FakeAudioCapturer> Create(Config config) {
    return webrtc::make_ref_counted<FakeAudioCapturer>(std::move(config));
  }

  FakeAudioCapturer(Config config);
  ~FakeAudioCapturer() override;

  // ビデオから円が一周したタイミングでビープ音を鳴らす
  void TriggerBeep();

  // AudioDeviceModule implementation
  int32_t ActiveAudioLayer(AudioLayer* audioLayer) const override;
  int32_t RegisterAudioCallback(webrtc::AudioTransport* audioCallback) override;
  int32_t Init() override;
  int32_t Terminate() override;
  bool Initialized() const override;

  // Device enumeration
  int16_t PlayoutDevices() override { return 0; }
  int16_t RecordingDevices() override { return 0; }
  int32_t PlayoutDeviceName(uint16_t index,
                            char name[webrtc::kAdmMaxDeviceNameSize],
                            char guid[webrtc::kAdmMaxGuidSize]) override {
    return 0;
  }
  int32_t RecordingDeviceName(uint16_t index,
                              char name[webrtc::kAdmMaxDeviceNameSize],
                              char guid[webrtc::kAdmMaxGuidSize]) override {
    return 0;
  }

  // Device selection
  int32_t SetPlayoutDevice(uint16_t index) override { return 0; }
  int32_t SetPlayoutDevice(WindowsDeviceType device) override { return 0; }
  int32_t SetRecordingDevice(uint16_t index) override { return 0; }
  int32_t SetRecordingDevice(WindowsDeviceType device) override { return 0; }

  // Audio transport initialization
  int32_t PlayoutIsAvailable(bool* available) override;
  int32_t InitPlayout() override { return 0; }
  bool PlayoutIsInitialized() const override { return false; }
  int32_t RecordingIsAvailable(bool* available) override;
  int32_t InitRecording() override;
  bool RecordingIsInitialized() const override;

  // Audio transport control
  int32_t StartPlayout() override { return 0; }
  int32_t StopPlayout() override { return 0; }
  bool Playing() const override { return false; }
  int32_t StartRecording() override;
  int32_t StopRecording() override;
  bool Recording() const override;

  // Audio mixer initialization
  int32_t InitSpeaker() override { return 0; }
  bool SpeakerIsInitialized() const override { return false; }
  int32_t InitMicrophone() override;
  bool MicrophoneIsInitialized() const override;

  // Volume controls
  int32_t SpeakerVolumeIsAvailable(bool* available) override {
    *available = false;
    return 0;
  }
  int32_t SetSpeakerVolume(uint32_t volume) override { return 0; }
  int32_t SpeakerVolume(uint32_t* volume) const override { return 0; }
  int32_t MaxSpeakerVolume(uint32_t* maxVolume) const override { return 0; }
  int32_t MinSpeakerVolume(uint32_t* minVolume) const override { return 0; }
  int32_t MicrophoneVolumeIsAvailable(bool* available) override {
    *available = false;
    return 0;
  }
  int32_t SetMicrophoneVolume(uint32_t volume) override { return 0; }
  int32_t MicrophoneVolume(uint32_t* volume) const override { return 0; }
  int32_t MaxMicrophoneVolume(uint32_t* maxVolume) const override { return 0; }
  int32_t MinMicrophoneVolume(uint32_t* minVolume) const override { return 0; }

  // Mute control
  int32_t SpeakerMuteIsAvailable(bool* available) override {
    *available = false;
    return 0;
  }
  int32_t SetSpeakerMute(bool enable) override { return 0; }
  int32_t SpeakerMute(bool* enabled) const override {
    *enabled = false;
    return 0;
  }
  int32_t MicrophoneMuteIsAvailable(bool* available) override {
    *available = false;
    return 0;
  }
  int32_t SetMicrophoneMute(bool enable) override { return 0; }
  int32_t MicrophoneMute(bool* enabled) const override { return 0; }

  // Stereo support
  int32_t StereoPlayoutIsAvailable(bool* available) const override {
    *available = false;
    return 0;
  }
  int32_t SetStereoPlayout(bool enable) override { return 0; }
  int32_t StereoPlayout(bool* enabled) const override {
    *enabled = false;
    return 0;
  }
  int32_t StereoRecordingIsAvailable(bool* available) const override {
    *available = false;
    return 0;
  }
  int32_t SetStereoRecording(bool enable) override { return 0; }
  int32_t StereoRecording(bool* enabled) const override {
    *enabled = false;
    return 0;
  }

  // Playout delay
  int32_t PlayoutDelay(uint16_t* delayMS) const override { return 0; }

  // Built-in effects (not supported)
  bool BuiltInAECIsAvailable() const override { return false; }
  bool BuiltInAGCIsAvailable() const override { return false; }
  bool BuiltInNSIsAvailable() const override { return false; }
  int32_t EnableBuiltInAEC(bool enable) override { return 0; }
  int32_t EnableBuiltInAGC(bool enable) override { return 0; }
  int32_t EnableBuiltInNS(bool enable) override { return 0; }

 private:
  void AudioThread();
  void GenerateBeep(std::vector<int16_t>& buffer, int samples);
  void GenerateSilence(std::vector<int16_t>& buffer, int samples);

  webrtc::Environment env_;
  Config config_;
  std::unique_ptr<webrtc::AudioDeviceBuffer> device_buffer_;
  std::unique_ptr<std::thread> audio_thread_;
  std::atomic<bool> stop_audio_thread_{false};
  std::atomic<bool> initialized_{false};
  std::atomic<bool> recording_initialized_{false};
  std::atomic<bool> microphone_initialized_{false};
  std::atomic<bool> is_recording_{false};

  // ビープ音制御
  std::mutex beep_mutex_;
  bool trigger_beep_ = false;
  int beep_samples_remaining_ = 0;
  double beep_phase_ = 0.0;           // sin波の位相
  const int beep_duration_ms_ = 100;  // ビープ音の長さ（ミリ秒）
  const int beep_frequency_ = 1000;   // ビープ音の周波数（Hz）
};

#endif  // USE_FAKE_CAPTURE_DEVICE

#endif  // RTC_FAKE_AUDIO_CAPTURER_H_