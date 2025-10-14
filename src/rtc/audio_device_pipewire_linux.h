#ifndef RTC_AUDIO_DEVICE_PIPEWIRE_LINUX_H_
#define RTC_AUDIO_DEVICE_PIPEWIRE_LINUX_H_

#include <memory>
#include <string>
#include <vector>

// WebRTC
#include <api/audio/audio_device.h>
#include <api/audio/audio_device_defines.h>
#include <api/sequence_checker.h>
#include <modules/audio_device/audio_device_buffer.h>
#include <modules/audio_device/audio_device_generic.h>
#include <rtc_base/event.h>
#include <rtc_base/platform_thread.h>
#include <rtc_base/synchronization/mutex.h>
#include <rtc_base/thread_annotations.h>

// PipeWire
#include <pipewire/pipewire.h>
#include <spa/param/audio/format-utils.h>
#include <spa/param/latency-utils.h>

namespace webrtc {

class AudioDeviceLinuxPipeWire : public AudioDeviceGeneric {
 public:
  AudioDeviceLinuxPipeWire();
  virtual ~AudioDeviceLinuxPipeWire();

  // AudioDeviceGeneric implementation
  int32_t ActiveAudioLayer(
      AudioDeviceModule::AudioLayer& audioLayer) const override;

  int32_t RegisterAudioCallback(AudioTransport* audioCallback);

  InitStatus Init() override;
  int32_t Terminate() RTC_LOCKS_EXCLUDED(mutex_) override;
  bool Initialized() const override;

  // Device enumeration
  int16_t PlayoutDevices() override;
  int16_t RecordingDevices() override;
  int32_t PlayoutDeviceName(uint16_t index,
                            char name[kAdmMaxDeviceNameSize],
                            char guid[kAdmMaxGuidSize]) override;
  int32_t RecordingDeviceName(uint16_t index,
                              char name[kAdmMaxDeviceNameSize],
                              char guid[kAdmMaxGuidSize]) override;

  // Device selection
  int32_t SetPlayoutDevice(uint16_t index) override;
  int32_t SetPlayoutDevice(
      AudioDeviceModule::WindowsDeviceType device) override;
  int32_t SetRecordingDevice(uint16_t index) override;
  int32_t SetRecordingDevice(
      AudioDeviceModule::WindowsDeviceType device) override;

  // Audio transport initialization
  int32_t PlayoutIsAvailable(bool& available) override;
  int32_t InitPlayout() RTC_LOCKS_EXCLUDED(mutex_) override;
  bool PlayoutIsInitialized() const override;
  int32_t RecordingIsAvailable(bool& available) override;
  int32_t InitRecording() override;
  bool RecordingIsInitialized() const override;

  // Audio transport control
  int32_t StartPlayout() RTC_LOCKS_EXCLUDED(mutex_) override;
  int32_t StopPlayout() RTC_LOCKS_EXCLUDED(mutex_) override;
  bool Playing() const override;
  int32_t StartRecording() RTC_LOCKS_EXCLUDED(mutex_) override;
  int32_t StopRecording() RTC_LOCKS_EXCLUDED(mutex_) override;
  bool Recording() const override;

  // Simplified implementations (not fully implemented)
  int32_t InitSpeaker() override { return 0; }
  bool SpeakerIsInitialized() const override { return true; }
  int32_t InitMicrophone() override { return 0; }
  bool MicrophoneIsInitialized() const override { return true; }
  int32_t SpeakerVolumeIsAvailable(bool& available) override {
    available = false;
    return 0;
  }
  int32_t SetSpeakerVolume(uint32_t volume) override { return -1; }
  int32_t SpeakerVolume(uint32_t& volume) const override { return -1; }
  int32_t MaxSpeakerVolume(uint32_t& maxVolume) const override { return -1; }
  int32_t MinSpeakerVolume(uint32_t& minVolume) const override { return -1; }
  int32_t MicrophoneVolumeIsAvailable(bool& available) override {
    available = false;
    return 0;
  }
  int32_t SetMicrophoneVolume(uint32_t volume) override { return -1; }
  int32_t MicrophoneVolume(uint32_t& volume) const override { return -1; }
  int32_t MaxMicrophoneVolume(uint32_t& maxVolume) const override {
    return -1;
  }
  int32_t MinMicrophoneVolume(uint32_t& minVolume) const override {
    return -1;
  }
  int32_t SpeakerMuteIsAvailable(bool& available) override {
    available = false;
    return 0;
  }
  int32_t SetSpeakerMute(bool enable) override { return -1; }
  int32_t SpeakerMute(bool& enabled) const override { return -1; }
  int32_t MicrophoneMuteIsAvailable(bool& available) override {
    available = false;
    return 0;
  }
  int32_t SetMicrophoneMute(bool enable) override { return -1; }
  int32_t MicrophoneMute(bool& enabled) const override { return -1; }
  int32_t StereoPlayoutIsAvailable(bool& available) override {
    available = true;
    return 0;
  }
  int32_t SetStereoPlayout(bool enable) override { return 0; }
  int32_t StereoPlayout(bool& enabled) const override {
    enabled = true;
    return 0;
  }
  int32_t StereoRecordingIsAvailable(bool& available) override {
    available = true;
    return 0;
  }
  int32_t SetStereoRecording(bool enable) override { return 0; }
  int32_t StereoRecording(bool& enabled) const override {
    enabled = true;
    return 0;
  }
  int32_t PlayoutDelay(uint16_t& delayMS) const
      RTC_LOCKS_EXCLUDED(mutex_) override;

  void AttachAudioBuffer(AudioDeviceBuffer* audioBuffer) override;

 private:
  struct DeviceInfo {
    uint32_t id;
    std::string name;
    std::string description;
  };

  void EnumerateDevices();
  static void OnRegistryGlobal(void* data,
                                uint32_t id,
                                uint32_t permissions,
                                const char* type,
                                uint32_t version,
                                const struct spa_dict* props);
  static void OnRegistryGlobalRemove(void* data, uint32_t id);

  // Recording stream callbacks
  static void OnRecStreamStateChanged(void* data,
                                      enum pw_stream_state old,
                                      enum pw_stream_state state,
                                      const char* error);
  static void OnRecStreamProcess(void* data);

  AudioDeviceBuffer* audio_buffer_;
  AudioTransport* audio_transport_;
  mutable Mutex mutex_;

  bool initialized_;
  bool recording_;
  bool playing_;
  bool rec_is_initialized_;
  bool play_is_initialized_;

  uint16_t input_device_index_;
  uint16_t output_device_index_;
  bool input_device_is_specified_;
  bool output_device_is_specified_;

  std::vector<DeviceInfo> input_devices_;
  std::vector<DeviceInfo> output_devices_;

  // PipeWire objects
  struct pw_thread_loop* pw_main_loop_;
  struct pw_context* pw_context_;
  struct pw_core* pw_core_;
  struct pw_registry* pw_registry_;
  struct spa_hook registry_listener_;

  struct pw_stream* play_stream_;
  struct pw_stream* rec_stream_;
  struct spa_hook play_stream_listener_;
  struct spa_hook rec_stream_listener_;

  SequenceChecker thread_checker_;

  // Audio format constants
  static constexpr int kSampleRate = 48000;
  static constexpr int kChannels = 2;
  static constexpr int kFramesPerBuffer = 480;  // 10ms at 48kHz

  // Recording buffer for accumulating samples to 10ms boundary
  std::unique_ptr<int16_t[]> recording_buffer_;
  size_t recording_buffer_size_;  // in samples (frames * channels)
  size_t recording_frames_in_buffer_;  // current frames stored in buffer
};

}  // namespace webrtc

#endif  // RTC_AUDIO_DEVICE_PIPEWIRE_LINUX_H_
