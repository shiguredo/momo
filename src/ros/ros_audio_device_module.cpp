/*
 *  Copyright (c) 2012 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "ros_audio_device_module.h"

#include "modules/audio_device/audio_device_config.h"
#include "modules/audio_device/audio_device_generic.h"
#include "rtc_base/checks.h"
#include "rtc_base/logging.h"
#include "rtc_base/refcount.h"
#include "rtc_base/refcountedobject.h"
#include "system_wrappers/include/metrics.h"

ROSAudioDeviceModule::ROSAudioDeviceModule()
{
  RTC_LOG(INFO) << "Current setting use Dolby Voice";
}

rtc::scoped_refptr<webrtc::AudioDeviceModule> ROSAudioDeviceModule::Create()
{
  RTC_LOG(INFO) << __FUNCTION__;
  return new rtc::RefCountedObject<ROSAudioDeviceModule>();
}

int32_t ROSAudioDeviceModule::AttachAudioBuffer()
{
  RTC_LOG(INFO) << __FUNCTION__;
  audio_device_->AttachAudioBuffer(audio_device_buffer_.get());
  return 0;
}

ROSAudioDeviceModule::~ROSAudioDeviceModule()
{
  RTC_LOG(INFO) << __FUNCTION__;
}

int32_t ROSAudioDeviceModule::ActiveAudioLayer(AudioLayer *audioLayer) const
{
  RTC_LOG(INFO) << __FUNCTION__;
  AudioLayer activeAudio;
  if (audio_device_->ActiveAudioLayer(activeAudio) == -1)
  {
    return -1;
  }
  *audioLayer = activeAudio;
  return 0;
}

int32_t ROSAudioDeviceModule::Init()
{
  RTC_LOG(INFO) << __FUNCTION__;
  if (initialized_)
    return 0;

  audio_device_buffer_.reset(new webrtc::AudioDeviceBuffer());
  audio_device_.reset(new ROSAudioDevice());
  RTC_CHECK(audio_device_);

  this->AttachAudioBuffer();

  webrtc::AudioDeviceGeneric::InitStatus status = audio_device_->Init();
  RTC_HISTOGRAM_ENUMERATION(
      "WebRTC.Audio.InitializationResult", static_cast<int>(status),
      static_cast<int>(webrtc::AudioDeviceGeneric::InitStatus::NUM_STATUSES));
  if (status != webrtc::AudioDeviceGeneric::InitStatus::OK)
  {
    RTC_LOG(LS_ERROR) << "Audio device initialization failed.";
    return -1;
  }
  initialized_ = true;
  return 0;
}

int32_t ROSAudioDeviceModule::Terminate()
{
  RTC_LOG(INFO) << __FUNCTION__;
  if (!initialized_)
    return 0;
  if (audio_device_->Terminate() == -1)
  {
    return -1;
  }
  initialized_ = false;
  return 0;
}

bool ROSAudioDeviceModule::Initialized() const
{
  RTC_LOG(INFO) << __FUNCTION__ << ": " << initialized_;
  return initialized_;
}

int32_t ROSAudioDeviceModule::InitSpeaker()
{
  RTC_LOG(INFO) << __FUNCTION__;
  CHECKinitialized_();
  return audio_device_->InitSpeaker();
}

int32_t ROSAudioDeviceModule::InitMicrophone()
{
  RTC_LOG(INFO) << __FUNCTION__;
  CHECKinitialized_();
  return audio_device_->InitMicrophone();
}

int32_t ROSAudioDeviceModule::SpeakerVolumeIsAvailable(bool *available)
{
  RTC_LOG(INFO) << __FUNCTION__;
  CHECKinitialized_();
  bool isAvailable = false;
  if (audio_device_->SpeakerVolumeIsAvailable(isAvailable) == -1)
  {
    return -1;
  }
  *available = isAvailable;
  RTC_LOG(INFO) << "output: " << isAvailable;
  return 0;
}

int32_t ROSAudioDeviceModule::SetSpeakerVolume(uint32_t volume)
{
  RTC_LOG(INFO) << __FUNCTION__ << "(" << volume << ")";
  CHECKinitialized_();
  return audio_device_->SetSpeakerVolume(volume);
}

int32_t ROSAudioDeviceModule::SpeakerVolume(uint32_t *volume) const
{
  RTC_LOG(INFO) << __FUNCTION__;
  CHECKinitialized_();
  uint32_t level = 0;
  if (audio_device_->SpeakerVolume(level) == -1)
  {
    return -1;
  }
  *volume = level;
  RTC_LOG(INFO) << "output: " << *volume;
  return 0;
}

bool ROSAudioDeviceModule::SpeakerIsInitialized() const
{
  RTC_LOG(INFO) << __FUNCTION__;
  CHECKinitialized__BOOL();
  bool isInitialized = audio_device_->SpeakerIsInitialized();
  RTC_LOG(INFO) << "output: " << isInitialized;
  return isInitialized;
}

bool ROSAudioDeviceModule::MicrophoneIsInitialized() const
{
  RTC_LOG(INFO) << __FUNCTION__;
  CHECKinitialized__BOOL();
  bool isInitialized = audio_device_->MicrophoneIsInitialized();
  RTC_LOG(INFO) << "output: " << isInitialized;
  return isInitialized;
}

int32_t ROSAudioDeviceModule::MaxSpeakerVolume(uint32_t *maxVolume) const
{
  CHECKinitialized_();
  uint32_t maxVol = 0;
  if (audio_device_->MaxSpeakerVolume(maxVol) == -1)
  {
    return -1;
  }
  *maxVolume = maxVol;
  return 0;
}

int32_t ROSAudioDeviceModule::MinSpeakerVolume(uint32_t *minVolume) const
{
  CHECKinitialized_();
  uint32_t minVol = 0;
  if (audio_device_->MinSpeakerVolume(minVol) == -1)
  {
    return -1;
  }
  *minVolume = minVol;
  return 0;
}

int32_t ROSAudioDeviceModule::SpeakerMuteIsAvailable(bool *available)
{
  RTC_LOG(INFO) << __FUNCTION__;
  CHECKinitialized_();
  bool isAvailable = false;
  if (audio_device_->SpeakerMuteIsAvailable(isAvailable) == -1)
  {
    return -1;
  }
  *available = isAvailable;
  RTC_LOG(INFO) << "output: " << isAvailable;
  return 0;
}

int32_t ROSAudioDeviceModule::SetSpeakerMute(bool enable)
{
  RTC_LOG(INFO) << __FUNCTION__ << "(" << enable << ")";
  CHECKinitialized_();
  return audio_device_->SetSpeakerMute(enable);
}

int32_t ROSAudioDeviceModule::SpeakerMute(bool *enabled) const
{
  RTC_LOG(INFO) << __FUNCTION__;
  CHECKinitialized_();
  bool muted = false;
  if (audio_device_->SpeakerMute(muted) == -1)
  {
    return -1;
  }
  *enabled = muted;
  RTC_LOG(INFO) << "output: " << muted;
  return 0;
}

int32_t ROSAudioDeviceModule::MicrophoneMuteIsAvailable(bool *available)
{
  RTC_LOG(INFO) << __FUNCTION__;
  CHECKinitialized_();
  bool isAvailable = false;
  if (audio_device_->MicrophoneMuteIsAvailable(isAvailable) == -1)
  {
    return -1;
  }
  *available = isAvailable;
  RTC_LOG(INFO) << "output: " << isAvailable;
  return 0;
}

int32_t ROSAudioDeviceModule::SetMicrophoneMute(bool enable)
{
  RTC_LOG(INFO) << __FUNCTION__ << "(" << enable << ")";
  CHECKinitialized_();
  return (audio_device_->SetMicrophoneMute(enable));
}

int32_t ROSAudioDeviceModule::MicrophoneMute(bool *enabled) const
{
  RTC_LOG(INFO) << __FUNCTION__;
  CHECKinitialized_();
  bool muted = false;
  if (audio_device_->MicrophoneMute(muted) == -1)
  {
    return -1;
  }
  *enabled = muted;
  RTC_LOG(INFO) << "output: " << muted;
  return 0;
}

int32_t ROSAudioDeviceModule::MicrophoneVolumeIsAvailable(bool *available)
{
  RTC_LOG(INFO) << __FUNCTION__;
  CHECKinitialized_();
  bool isAvailable = false;
  if (audio_device_->MicrophoneVolumeIsAvailable(isAvailable) == -1)
  {
    return -1;
  }
  *available = isAvailable;
  RTC_LOG(INFO) << "output: " << isAvailable;
  return 0;
}

int32_t ROSAudioDeviceModule::SetMicrophoneVolume(uint32_t volume)
{
  RTC_LOG(INFO) << __FUNCTION__ << "(" << volume << ")";
  CHECKinitialized_();
  return (audio_device_->SetMicrophoneVolume(volume));
}

int32_t ROSAudioDeviceModule::MicrophoneVolume(uint32_t *volume) const
{
  RTC_LOG(INFO) << __FUNCTION__;
  CHECKinitialized_();
  uint32_t level = 0;
  if (audio_device_->MicrophoneVolume(level) == -1)
  {
    return -1;
  }
  *volume = level;
  RTC_LOG(INFO) << "output: " << *volume;
  return 0;
}

int32_t ROSAudioDeviceModule::StereoRecordingIsAvailable(
    bool *available) const
{
  RTC_LOG(INFO) << __FUNCTION__;
  CHECKinitialized_();
  bool isAvailable = false;
  if (audio_device_->StereoRecordingIsAvailable(isAvailable) == -1)
  {
    return -1;
  }
  *available = isAvailable;
  RTC_LOG(INFO) << "output: " << isAvailable;
  return 0;
}

int32_t ROSAudioDeviceModule::SetStereoRecording(bool enable)
{
  RTC_LOG(INFO) << __FUNCTION__ << "(" << enable << ")";
  CHECKinitialized_();
  if (audio_device_->RecordingIsInitialized())
  {
    RTC_LOG(WARNING) << "recording in stereo is not supported";
    return -1;
  }
  if (audio_device_->SetStereoRecording(enable) == -1)
  {
    RTC_LOG(WARNING) << "failed to change stereo recording";
    return -1;
  }
  int8_t nChannels(1);
  if (enable)
  {
    nChannels = 2;
  }
  audio_device_buffer_.get()->SetRecordingChannels(nChannels);
  return 0;
}

int32_t ROSAudioDeviceModule::StereoRecording(bool *enabled) const
{
  RTC_LOG(INFO) << __FUNCTION__;
  CHECKinitialized_();
  bool stereo = false;
  if (audio_device_->StereoRecording(stereo) == -1)
  {
    return -1;
  }
  *enabled = stereo;
  RTC_LOG(INFO) << "output: " << stereo;
  return 0;
}

int32_t ROSAudioDeviceModule::StereoPlayoutIsAvailable(bool *available) const
{
  RTC_LOG(INFO) << __FUNCTION__;
  CHECKinitialized_();
  bool isAvailable = false;
  if (audio_device_->StereoPlayoutIsAvailable(isAvailable) == -1)
  {
    return -1;
  }
  *available = isAvailable;
  RTC_LOG(INFO) << "output: " << isAvailable;
  return 0;
}

int32_t ROSAudioDeviceModule::SetStereoPlayout(bool enable)
{
  RTC_LOG(INFO) << __FUNCTION__ << "(" << enable << ")";
  CHECKinitialized_();
  if (audio_device_->PlayoutIsInitialized())
  {
    RTC_LOG(LERROR)
        << "unable to set stereo mode while playing side is initialized";
    return -1;
  }
  if (audio_device_->SetStereoPlayout(enable))
  {
    RTC_LOG(WARNING) << "stereo playout is not supported";
    return -1;
  }
  int8_t nChannels(1);
  if (enable)
  {
    nChannels = 2;
  }
  audio_device_buffer_.get()->SetPlayoutChannels(nChannels);
  return 0;
}

int32_t ROSAudioDeviceModule::StereoPlayout(bool *enabled) const
{
  RTC_LOG(INFO) << __FUNCTION__;
  CHECKinitialized_();
  bool stereo = false;
  if (audio_device_->StereoPlayout(stereo) == -1)
  {
    return -1;
  }
  *enabled = stereo;
  RTC_LOG(INFO) << "output: " << stereo;
  return 0;
}

int32_t ROSAudioDeviceModule::PlayoutIsAvailable(bool *available)
{
  RTC_LOG(INFO) << __FUNCTION__;
  CHECKinitialized_();
  bool isAvailable = false;
  if (audio_device_->PlayoutIsAvailable(isAvailable) == -1)
  {
    return -1;
  }
  *available = isAvailable;
  RTC_LOG(INFO) << "output: " << isAvailable;
  return 0;
}

int32_t ROSAudioDeviceModule::RecordingIsAvailable(bool *available)
{
  RTC_LOG(INFO) << __FUNCTION__;
  CHECKinitialized_();
  bool isAvailable = false;
  if (audio_device_->RecordingIsAvailable(isAvailable) == -1)
  {
    return -1;
  }
  *available = isAvailable;
  RTC_LOG(INFO) << "output: " << isAvailable;
  return 0;
}

int32_t ROSAudioDeviceModule::MaxMicrophoneVolume(uint32_t *maxVolume) const
{
  CHECKinitialized_();
  uint32_t maxVol(0);
  if (audio_device_->MaxMicrophoneVolume(maxVol) == -1)
  {
    return -1;
  }
  *maxVolume = maxVol;
  return 0;
}

int32_t ROSAudioDeviceModule::MinMicrophoneVolume(uint32_t *minVolume) const
{
  CHECKinitialized_();
  uint32_t minVol(0);
  if (audio_device_->MinMicrophoneVolume(minVol) == -1)
  {
    return -1;
  }
  *minVolume = minVol;
  return 0;
}

int16_t ROSAudioDeviceModule::PlayoutDevices()
{
  RTC_LOG(INFO) << __FUNCTION__;
  CHECKinitialized_();
  uint16_t nPlayoutDevices = audio_device_->PlayoutDevices();
  RTC_LOG(INFO) << "output: " << nPlayoutDevices;
  return (int16_t)(nPlayoutDevices);
}

int32_t ROSAudioDeviceModule::SetPlayoutDevice(uint16_t index)
{
  RTC_LOG(INFO) << __FUNCTION__ << "(" << index << ")";
  CHECKinitialized_();
  return audio_device_->SetPlayoutDevice(index);
}

int32_t ROSAudioDeviceModule::SetPlayoutDevice(WindowsDeviceType device)
{
  RTC_LOG(INFO) << __FUNCTION__;
  CHECKinitialized_();
  return audio_device_->SetPlayoutDevice(device);
}

int32_t ROSAudioDeviceModule::PlayoutDeviceName(
    uint16_t index,
    char name[webrtc::kAdmMaxDeviceNameSize],
    char guid[webrtc::kAdmMaxGuidSize])
{
  RTC_LOG(INFO) << __FUNCTION__ << "(" << index << ", ...)";
  CHECKinitialized_();
  if (name == NULL)
  {
    return -1;
  }
  if (audio_device_->PlayoutDeviceName(index, name, guid) == -1)
  {
    return -1;
  }
  if (name != NULL)
  {
    RTC_LOG(INFO) << "output: name = " << name;
  }
  if (guid != NULL)
  {
    RTC_LOG(INFO) << "output: guid = " << guid;
  }
  return 0;
}

int32_t ROSAudioDeviceModule::RecordingDeviceName(
    uint16_t index,
    char name[webrtc::kAdmMaxDeviceNameSize],
    char guid[webrtc::kAdmMaxGuidSize])
{
  RTC_LOG(INFO) << __FUNCTION__ << "(" << index << ", ...)";
  CHECKinitialized_();
  if (name == NULL)
  {
    return -1;
  }
  if (audio_device_->RecordingDeviceName(index, name, guid) == -1)
  {
    return -1;
  }
  if (name != NULL)
  {
    RTC_LOG(INFO) << "output: name = " << name;
  }
  if (guid != NULL)
  {
    RTC_LOG(INFO) << "output: guid = " << guid;
  }
  return 0;
}

int16_t ROSAudioDeviceModule::RecordingDevices()
{
  RTC_LOG(INFO) << __FUNCTION__;
  CHECKinitialized_();
  uint16_t nRecordingDevices = audio_device_->RecordingDevices();
  RTC_LOG(INFO) << "output: " << nRecordingDevices;
  return (int16_t)nRecordingDevices;
}

int32_t ROSAudioDeviceModule::SetRecordingDevice(uint16_t index)
{
  RTC_LOG(INFO) << __FUNCTION__ << "(" << index << ")";
  CHECKinitialized_();
  return audio_device_->SetRecordingDevice(index);
}

int32_t ROSAudioDeviceModule::SetRecordingDevice(WindowsDeviceType device)
{
  RTC_LOG(INFO) << __FUNCTION__;
  CHECKinitialized_();
  return audio_device_->SetRecordingDevice(device);
}

int32_t ROSAudioDeviceModule::InitPlayout()
{
  RTC_LOG(INFO) << __FUNCTION__;
  CHECKinitialized_();
  if (PlayoutIsInitialized())
  {
    return 0;
  }
  int32_t result = audio_device_->InitPlayout();
  RTC_LOG(INFO) << "output: " << result;
  RTC_HISTOGRAM_BOOLEAN("WebRTC.Audio.InitPlayoutSuccess",
                        static_cast<int>(result == 0));
  return result;
}

int32_t ROSAudioDeviceModule::InitRecording()
{
  RTC_LOG(INFO) << __FUNCTION__;
  CHECKinitialized_();
  if (RecordingIsInitialized())
  {
    return 0;
  }
  int32_t result = audio_device_->InitRecording();
  RTC_LOG(INFO) << "output: " << result;
  RTC_HISTOGRAM_BOOLEAN("WebRTC.Audio.InitRecordingSuccess",
                        static_cast<int>(result == 0));
  return result;
}

bool ROSAudioDeviceModule::PlayoutIsInitialized() const
{
  RTC_LOG(INFO) << __FUNCTION__;
  CHECKinitialized__BOOL();
  return audio_device_->PlayoutIsInitialized();
}

bool ROSAudioDeviceModule::RecordingIsInitialized() const
{
  RTC_LOG(INFO) << __FUNCTION__;
  CHECKinitialized__BOOL();
  return audio_device_->RecordingIsInitialized();
}

int32_t ROSAudioDeviceModule::StartPlayout()
{
  RTC_LOG(INFO) << __FUNCTION__;
  CHECKinitialized_();
  if (Playing())
  {
    return 0;
  }
  audio_device_buffer_.get()->StartPlayout();
  int32_t result = audio_device_->StartPlayout();
  RTC_LOG(INFO) << "output: " << result;
  RTC_HISTOGRAM_BOOLEAN("WebRTC.Audio.StartPlayoutSuccess",
                        static_cast<int>(result == 0));
  return result;
}

int32_t ROSAudioDeviceModule::StopPlayout()
{
  RTC_LOG(INFO) << __FUNCTION__;
  CHECKinitialized_();
  int32_t result = audio_device_->StopPlayout();
  audio_device_buffer_.get()->StopPlayout();
  RTC_LOG(INFO) << "output: " << result;
  RTC_HISTOGRAM_BOOLEAN("WebRTC.Audio.StopPlayoutSuccess",
                        static_cast<int>(result == 0));
  return result;
}

bool ROSAudioDeviceModule::Playing() const
{
  RTC_LOG(INFO) << __FUNCTION__;
  CHECKinitialized__BOOL();
  return audio_device_->Playing();
}

int32_t ROSAudioDeviceModule::StartRecording()
{
  RTC_LOG(INFO) << __FUNCTION__;
  CHECKinitialized_();
  if (Recording())
  {
    return 0;
  }
  audio_device_buffer_.get()->StartRecording();
  int32_t result = audio_device_->StartRecording();
  RTC_LOG(INFO) << "output: " << result;
  RTC_HISTOGRAM_BOOLEAN("WebRTC.Audio.StartRecordingSuccess",
                        static_cast<int>(result == 0));
  return result;
}

int32_t ROSAudioDeviceModule::StopRecording()
{
  RTC_LOG(INFO) << __FUNCTION__;
  CHECKinitialized_();
  int32_t result = audio_device_->StopRecording();
  audio_device_buffer_.get()->StopRecording();
  RTC_LOG(INFO) << "output: " << result;
  RTC_HISTOGRAM_BOOLEAN("WebRTC.Audio.StopRecordingSuccess",
                        static_cast<int>(result == 0));
  return result;
}

bool ROSAudioDeviceModule::Recording() const
{
  RTC_LOG(INFO) << __FUNCTION__;
  CHECKinitialized__BOOL();
  return audio_device_->Recording();
}

int32_t ROSAudioDeviceModule::RegisterAudioCallback(
    webrtc::AudioTransport *audioCallback)
{
  RTC_LOG(INFO) << __FUNCTION__;
  return audio_device_buffer_.get()->RegisterAudioCallback(audioCallback);
}

int32_t ROSAudioDeviceModule::PlayoutDelay(uint16_t *delayMS) const
{
  CHECKinitialized_();
  uint16_t delay = 0;
  if (audio_device_->PlayoutDelay(delay) == -1)
  {
    RTC_LOG(LERROR) << "failed to retrieve the playout delay";
    return -1;
  }
  *delayMS = delay;
  return 0;
}

bool ROSAudioDeviceModule::BuiltInAECIsAvailable() const
{
  return true;
}

int32_t ROSAudioDeviceModule::EnableBuiltInAEC(bool enable)
{
  return 0;
}

bool ROSAudioDeviceModule::BuiltInAGCIsAvailable() const
{
  return true;
}

int32_t ROSAudioDeviceModule::EnableBuiltInAGC(bool enable)
{
  return 0;
}

bool ROSAudioDeviceModule::BuiltInNSIsAvailable() const
{
  return true;
}

int32_t ROSAudioDeviceModule::EnableBuiltInNS(bool enable)
{
  return 0;
}

#if defined(WEBRTC_IOS)
int ROSAudioDeviceModule::GetPlayoutAudioParameters(
    webrtc::AudioParameters *params) const
{
  RTC_LOG(INFO) << __FUNCTION__;
  int r = audio_device_->GetPlayoutAudioParameters(params);
  RTC_LOG(INFO) << "output: " << r;
  return r;
}

int ROSAudioDeviceModule::GetRecordAudioParameters(
    webrtc::AudioParameters *params) const
{
  RTC_LOG(INFO) << __FUNCTION__;
  int r = audio_device_->GetRecordAudioParameters(params);
  RTC_LOG(INFO) << "output: " << r;
  return r;
}
#endif // WEBRTC_IOS