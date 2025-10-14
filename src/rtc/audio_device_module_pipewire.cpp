#include "audio_device_module_pipewire.h"

#include "audio_device_pipewire_linux.h"

#include <optional>

#include <rtc_base/logging.h>
#include <rtc_base/ref_count.h>
#include <rtc_base/ref_counted_object.h>

namespace webrtc {

// AudioDeviceModule を実装する wrapper クラス
class AudioDeviceModulePipeWire : public AudioDeviceModule {
 public:
  AudioDeviceModulePipeWire() : impl_(new AudioDeviceLinuxPipeWire()) {}
  ~AudioDeviceModulePipeWire() override { delete impl_; }

  // RefCountInterface implementation
  void AddRef() const override { ref_count_.IncRef(); }
  webrtc::RefCountReleaseStatus Release() const override {
    const auto status = ref_count_.DecRef();
    if (status == webrtc::RefCountReleaseStatus::kDroppedLastRef) {
      delete this;
    }
    return status;
  }

  // AudioDeviceModule implementation
  int32_t ActiveAudioLayer(AudioLayer* audioLayer) const override {
    AudioLayer layer;
    int32_t result = impl_->ActiveAudioLayer(layer);
    if (audioLayer) {
      *audioLayer = layer;
    }
    return result;
  }

  int32_t RegisterAudioCallback(AudioTransport* audioCallback) override {
    return impl_->RegisterAudioCallback(audioCallback);
  }

  int32_t Init() override {
    RTC_LOG(LS_INFO) << "AudioDeviceModulePipeWire::Init() called";
    AudioDeviceGeneric::InitStatus status = impl_->Init();
    RTC_LOG(LS_INFO) << "AudioDeviceLinuxPipeWire::Init() returned status: "
                     << static_cast<int>(status);
    return status == AudioDeviceGeneric::InitStatus::OK ? 0 : -1;
  }

  int32_t Terminate() override { return impl_->Terminate(); }

  bool Initialized() const override { return impl_->Initialized(); }

  int16_t PlayoutDevices() override { return impl_->PlayoutDevices(); }

  int16_t RecordingDevices() override { return impl_->RecordingDevices(); }

  int32_t PlayoutDeviceName(uint16_t index,
                            char name[kAdmMaxDeviceNameSize],
                            char guid[kAdmMaxGuidSize]) override {
    return impl_->PlayoutDeviceName(index, name, guid);
  }

  int32_t RecordingDeviceName(uint16_t index,
                              char name[kAdmMaxDeviceNameSize],
                              char guid[kAdmMaxGuidSize]) override {
    return impl_->RecordingDeviceName(index, name, guid);
  }

  int32_t SetPlayoutDevice(uint16_t index) override {
    return impl_->SetPlayoutDevice(index);
  }

  int32_t SetPlayoutDevice(WindowsDeviceType device) override {
    return impl_->SetPlayoutDevice(device);
  }

  int32_t SetRecordingDevice(uint16_t index) override {
    return impl_->SetRecordingDevice(index);
  }

  int32_t SetRecordingDevice(WindowsDeviceType device) override {
    return impl_->SetRecordingDevice(device);
  }

  int32_t PlayoutIsAvailable(bool* available) override {
    return impl_->PlayoutIsAvailable(*available);
  }

  int32_t InitPlayout() override { return impl_->InitPlayout(); }

  bool PlayoutIsInitialized() const override {
    return impl_->PlayoutIsInitialized();
  }

  int32_t RecordingIsAvailable(bool* available) override {
    return impl_->RecordingIsAvailable(*available);
  }

  int32_t InitRecording() override { return impl_->InitRecording(); }

  bool RecordingIsInitialized() const override {
    return impl_->RecordingIsInitialized();
  }

  int32_t StartPlayout() override { return impl_->StartPlayout(); }

  int32_t StopPlayout() override { return impl_->StopPlayout(); }

  bool Playing() const override { return impl_->Playing(); }

  int32_t StartRecording() override { return impl_->StartRecording(); }

  int32_t StopRecording() override { return impl_->StopRecording(); }

  bool Recording() const override { return impl_->Recording(); }

  int32_t InitSpeaker() override { return impl_->InitSpeaker(); }

  bool SpeakerIsInitialized() const override {
    return impl_->SpeakerIsInitialized();
  }

  int32_t InitMicrophone() override { return impl_->InitMicrophone(); }

  bool MicrophoneIsInitialized() const override {
    return impl_->MicrophoneIsInitialized();
  }

  int32_t SpeakerVolumeIsAvailable(bool* available) override {
    return impl_->SpeakerVolumeIsAvailable(*available);
  }

  int32_t SetSpeakerVolume(uint32_t volume) override {
    return impl_->SetSpeakerVolume(volume);
  }

  int32_t SpeakerVolume(uint32_t* volume) const override {
    return impl_->SpeakerVolume(*volume);
  }

  int32_t MaxSpeakerVolume(uint32_t* maxVolume) const override {
    return impl_->MaxSpeakerVolume(*maxVolume);
  }

  int32_t MinSpeakerVolume(uint32_t* minVolume) const override {
    return impl_->MinSpeakerVolume(*minVolume);
  }

  int32_t MicrophoneVolumeIsAvailable(bool* available) override {
    return impl_->MicrophoneVolumeIsAvailable(*available);
  }

  int32_t SetMicrophoneVolume(uint32_t volume) override {
    return impl_->SetMicrophoneVolume(volume);
  }

  int32_t MicrophoneVolume(uint32_t* volume) const override {
    return impl_->MicrophoneVolume(*volume);
  }

  int32_t MaxMicrophoneVolume(uint32_t* maxVolume) const override {
    return impl_->MaxMicrophoneVolume(*maxVolume);
  }

  int32_t MinMicrophoneVolume(uint32_t* minVolume) const override {
    return impl_->MinMicrophoneVolume(*minVolume);
  }

  int32_t SpeakerMuteIsAvailable(bool* available) override {
    return impl_->SpeakerMuteIsAvailable(*available);
  }

  int32_t SetSpeakerMute(bool enable) override {
    return impl_->SetSpeakerMute(enable);
  }

  int32_t SpeakerMute(bool* enabled) const override {
    return impl_->SpeakerMute(*enabled);
  }

  int32_t MicrophoneMuteIsAvailable(bool* available) override {
    return impl_->MicrophoneMuteIsAvailable(*available);
  }

  int32_t SetMicrophoneMute(bool enable) override {
    return impl_->SetMicrophoneMute(enable);
  }

  int32_t MicrophoneMute(bool* enabled) const override {
    return impl_->MicrophoneMute(*enabled);
  }

  int32_t StereoPlayoutIsAvailable(bool* available) const override {
    return impl_->StereoPlayoutIsAvailable(*available);
  }

  int32_t SetStereoPlayout(bool enable) override {
    return impl_->SetStereoPlayout(enable);
  }

  int32_t StereoPlayout(bool* enabled) const override {
    return impl_->StereoPlayout(*enabled);
  }

  int32_t StereoRecordingIsAvailable(bool* available) const override {
    return impl_->StereoRecordingIsAvailable(*available);
  }

  int32_t SetStereoRecording(bool enable) override {
    return impl_->SetStereoRecording(enable);
  }

  int32_t StereoRecording(bool* enabled) const override {
    return impl_->StereoRecording(*enabled);
  }

  int32_t PlayoutDelay(uint16_t* delayMS) const override {
    return impl_->PlayoutDelay(*delayMS);
  }

  bool BuiltInAECIsAvailable() const override { return false; }

  bool BuiltInAGCIsAvailable() const override { return false; }

  bool BuiltInNSIsAvailable() const override { return false; }

  int32_t EnableBuiltInAEC(bool enable) override { return -1; }

  int32_t EnableBuiltInAGC(bool enable) override { return -1; }

  int32_t EnableBuiltInNS(bool enable) override { return -1; }

#if defined(WEBRTC_IOS)
  int GetPlayoutAudioParameters(AudioParameters* params) const override {
    return -1;
  }

  int GetRecordAudioParameters(AudioParameters* params) const override {
    return -1;
  }
#endif

  std::optional<Stats> GetStats() const override {
    return std::nullopt;
  }

 private:
  mutable webrtc::webrtc_impl::RefCounter ref_count_{0};
  AudioDeviceLinuxPipeWire* impl_;
};

webrtc::scoped_refptr<AudioDeviceModule> CreatePipeWireAudioDeviceModule() {
  RTC_LOG(LS_INFO) << "CreatePipeWireAudioDeviceModule() called";
  auto* module = new AudioDeviceModulePipeWire();
  RTC_LOG(LS_INFO) << "AudioDeviceModulePipeWire created: " << module;
  return webrtc::scoped_refptr<AudioDeviceModule>(module);
}

}  // namespace webrtc
