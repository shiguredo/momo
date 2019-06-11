#ifndef ROS_AUDIO_DEVICE_MODULE_H_
#define ROS_AUDIO_DEVICE_MODULE_H_

#include <memory>

#include "api/task_queue/task_queue_factory.h"
#include "modules/audio_device/include/audio_device.h"

#include "ros_audio_device.h"
#include "connection_settings.h"

#define CHECKinitialized_() \
  {                         \
    if (!initialized_)      \
    {                       \
      return -1;            \
    }                       \
  }

#define CHECKinitialized__BOOL() \
  {                              \
    if (!initialized_)           \
    {                            \
      return false;              \
    }                            \
  }

class ROSAudioDeviceModule : public webrtc::AudioDeviceModule
{
public:
  static rtc::scoped_refptr<webrtc::AudioDeviceModule> Create(ConnectionSettings conn_settings,
                                                              webrtc::TaskQueueFactory* task_queue_factory);
  int32_t AttachAudioBuffer();

  ROSAudioDeviceModule(ConnectionSettings conn_settings,
                       webrtc::TaskQueueFactory* task_queue_factory);
  ~ROSAudioDeviceModule() override;

  // Retrieve the currently utilized audio layer
  int32_t ActiveAudioLayer(AudioLayer *audioLayer) const override;

  // Full-duplex transportation of PCM audio
  int32_t RegisterAudioCallback(webrtc::AudioTransport *audioCallback) override;

  // Main initializaton and termination
  int32_t Init() override;
  int32_t Terminate() override;
  bool Initialized() const override;

  // Device enumeration
  int16_t PlayoutDevices() override;
  int16_t RecordingDevices() override;
  int32_t PlayoutDeviceName(uint16_t index,
                            char name[webrtc::kAdmMaxDeviceNameSize],
                            char guid[webrtc::kAdmMaxGuidSize]) override;
  int32_t RecordingDeviceName(uint16_t index,
                              char name[webrtc::kAdmMaxDeviceNameSize],
                              char guid[webrtc::kAdmMaxGuidSize]) override;

  // Device selection
  int32_t SetPlayoutDevice(uint16_t index) override;
  int32_t SetPlayoutDevice(WindowsDeviceType device) override;
  int32_t SetRecordingDevice(uint16_t index) override;
  int32_t SetRecordingDevice(WindowsDeviceType device) override;

  // Audio transport initialization
  int32_t PlayoutIsAvailable(bool *available) override;
  int32_t InitPlayout() override;
  bool PlayoutIsInitialized() const override;
  int32_t RecordingIsAvailable(bool *available) override;
  int32_t InitRecording() override;
  bool RecordingIsInitialized() const override;

  // Audio transport control
  int32_t StartPlayout() override;
  int32_t StopPlayout() override;
  bool Playing() const override;
  int32_t StartRecording() override;
  int32_t StopRecording() override;
  bool Recording() const override;

  // Audio mixer initialization
  int32_t InitSpeaker() override;
  bool SpeakerIsInitialized() const override;
  int32_t InitMicrophone() override;
  bool MicrophoneIsInitialized() const override;

  // Speaker volume controls
  int32_t SpeakerVolumeIsAvailable(bool *available) override;
  int32_t SetSpeakerVolume(uint32_t volume) override;
  int32_t SpeakerVolume(uint32_t *volume) const override;
  int32_t MaxSpeakerVolume(uint32_t *maxVolume) const override;
  int32_t MinSpeakerVolume(uint32_t *minVolume) const override;

  // Microphone volume controls
  int32_t MicrophoneVolumeIsAvailable(bool *available) override;
  int32_t SetMicrophoneVolume(uint32_t volume) override;
  int32_t MicrophoneVolume(uint32_t *volume) const override;
  int32_t MaxMicrophoneVolume(uint32_t *maxVolume) const override;
  int32_t MinMicrophoneVolume(uint32_t *minVolume) const override;

  // Speaker mute control
  int32_t SpeakerMuteIsAvailable(bool *available) override;
  int32_t SetSpeakerMute(bool enable) override;
  int32_t SpeakerMute(bool *enabled) const override;

  // Microphone mute control
  int32_t MicrophoneMuteIsAvailable(bool *available) override;
  int32_t SetMicrophoneMute(bool enable) override;
  int32_t MicrophoneMute(bool *enabled) const override;

  // Stereo support
  int32_t StereoPlayoutIsAvailable(bool *available) const override;
  int32_t SetStereoPlayout(bool enable) override;
  int32_t StereoPlayout(bool *enabled) const override;
  int32_t StereoRecordingIsAvailable(bool *available) const override;
  int32_t SetStereoRecording(bool enable) override;
  int32_t StereoRecording(bool *enabled) const override;

  // Delay information and control
  int32_t PlayoutDelay(uint16_t *delayMS) const override;

  bool BuiltInAECIsAvailable() const override;
  int32_t EnableBuiltInAEC(bool enable) override;
  bool BuiltInAGCIsAvailable() const override;
  int32_t EnableBuiltInAGC(bool enable) override;
  bool BuiltInNSIsAvailable() const override;
  int32_t EnableBuiltInNS(bool enable) override;

#if defined(WEBRTC_IOS)
  int GetPlayoutAudioParameters(webrtc::AudioParameters *params) const override;
  int GetRecordAudioParameters(webrtc::AudioParameters *params) const override;
#endif // WEBRTC_IOS
private:
  ConnectionSettings _conn_settings;
  webrtc::TaskQueueFactory* task_queue_factory_;
  bool initialized_ = false;
  std::unique_ptr<ROSAudioDevice> audio_device_;
  std::unique_ptr<webrtc::AudioDeviceBuffer> audio_device_buffer_;
};

#endif