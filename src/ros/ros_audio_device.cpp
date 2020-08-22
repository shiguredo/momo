/*
 *  Copyright (c) 2014 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "ros_audio_device.h"

#include <rtc_base/checks.h>
#include <rtc_base/logging.h>
#include <system_wrappers/include/sleep.h>

const int kPlayoutFixedSampleRate = 8000;
const size_t kPlayoutNumChannels = 1;
const size_t kPlayoutBufferSize =
    kPlayoutFixedSampleRate / 100 * kPlayoutNumChannels * 2;

ROSAudioDevice::ROSAudioDevice(ROSAudioDeviceConfig config)
    : _config(std::move(config)),
      _ptrAudioBuffer(NULL),
      _recordingBuffer(NULL),
      _playoutBuffer(NULL),
      _recordingFramesLeft(0),
      _playoutFramesLeft(0),
      _recordingBufferSizeIn10MS(0),
      _recordingFramesIn10MS(0),
      _writtenBufferSize(0),
      _playoutFramesIn10MS(0),
      _playing(false),
      _recording(false),
      _lastCallPlayoutMillis(0),
      _lastCallRecordMillis(0) {}

ROSAudioDevice::~ROSAudioDevice() {
  Terminate();
}

int32_t ROSAudioDevice::ActiveAudioLayer(
    webrtc::AudioDeviceModule::AudioLayer& audioLayer) const {
  return -1;
}

webrtc::AudioDeviceGeneric::InitStatus ROSAudioDevice::Init() {
  return webrtc::AudioDeviceGeneric::InitStatus::OK;
}

int32_t ROSAudioDevice::Terminate() {
  StopRecording();
  return 0;
}

bool ROSAudioDevice::Initialized() const {
  return true;
}

int16_t ROSAudioDevice::PlayoutDevices() {
  return 1;
}

int16_t ROSAudioDevice::RecordingDevices() {
  return 1;
}

int32_t ROSAudioDevice::PlayoutDeviceName(
    uint16_t index,
    char name[webrtc::kAdmMaxDeviceNameSize],
    char guid[webrtc::kAdmMaxGuidSize]) {
  const char* kName = "dummy_device";
  const char* kGuid = "dummy_device_unique_id";
  if (index < 1) {
    memset(name, 0, webrtc::kAdmMaxDeviceNameSize);
    memset(guid, 0, webrtc::kAdmMaxGuidSize);
    memcpy(name, kName, strlen(kName));
    memcpy(guid, kGuid, strlen(guid));
    return 0;
  }
  return -1;
}

int32_t ROSAudioDevice::RecordingDeviceName(
    uint16_t index,
    char name[webrtc::kAdmMaxDeviceNameSize],
    char guid[webrtc::kAdmMaxGuidSize]) {
  const char* kName = "dummy_device";
  const char* kGuid = "dummy_device_unique_id";
  if (index < 1) {
    memset(name, 0, webrtc::kAdmMaxDeviceNameSize);
    memset(guid, 0, webrtc::kAdmMaxGuidSize);
    memcpy(name, kName, strlen(kName));
    memcpy(guid, kGuid, strlen(guid));
    return 0;
  }
  return -1;
}

int32_t ROSAudioDevice::SetPlayoutDevice(uint16_t index) {
  if (index == 0) {
    _playout_index = index;
    return 0;
  }
  return -1;
}

int32_t ROSAudioDevice::SetPlayoutDevice(
    webrtc::AudioDeviceModule::WindowsDeviceType device) {
  return -1;
}

int32_t ROSAudioDevice::SetRecordingDevice(uint16_t index) {
  if (index == 0) {
    _record_index = index;
    return _record_index;
  }
  return -1;
}

int32_t ROSAudioDevice::SetRecordingDevice(
    webrtc::AudioDeviceModule::WindowsDeviceType device) {
  return -1;
}

int32_t ROSAudioDevice::PlayoutIsAvailable(bool& available) {
  if (_playout_index == 0) {
    available = true;
    return _playout_index;
  }
  available = false;
  return -1;
}

int32_t ROSAudioDevice::InitPlayout() {
  rtc::CritScope lock(&_critSect);

  if (_playing) {
    return -1;
  }

  _playoutFramesIn10MS = static_cast<size_t>(kPlayoutFixedSampleRate / 100);

  if (_ptrAudioBuffer) {
    _ptrAudioBuffer->SetPlayoutSampleRate(kPlayoutFixedSampleRate);
    _ptrAudioBuffer->SetPlayoutChannels(kPlayoutNumChannels);
  }
  return 0;
}

bool ROSAudioDevice::PlayoutIsInitialized() const {
  return _playoutFramesIn10MS != 0;
}

int32_t ROSAudioDevice::RecordingIsAvailable(bool& available) {
  if (_record_index == 0) {
    available = true;
    return _record_index;
  }
  available = false;
  return -1;
}

int32_t ROSAudioDevice::InitRecording() {
  rtc::CritScope lock(&_critSect);

  if (_recording) {
    return -1;
  }

  _recordingFramesIn10MS = static_cast<size_t>(_config.audio_topic_rate / 100);

  if (_ptrAudioBuffer) {
    _ptrAudioBuffer->SetRecordingSampleRate(_config.audio_topic_rate);
    _ptrAudioBuffer->SetRecordingChannels(_config.audio_topic_ch);
  }
  return 0;
}

bool ROSAudioDevice::RecordingIsInitialized() const {
  return _recordingFramesIn10MS != 0;
}

int32_t ROSAudioDevice::StartPlayout() {
  if (_playing) {
    return 0;
  }

  _playing = true;
  _playoutFramesLeft = 0;

  if (!_playoutBuffer) {
    _playoutBuffer = new int8_t[kPlayoutBufferSize];
  }
  if (!_playoutBuffer) {
    _playing = false;
    return -1;
  }

  _ptrThreadPlay.reset(new rtc::PlatformThread(
      PlayThreadFunc, this, "webrtc_audio_module_play_thread",
      rtc::kRealtimePriority));
  _ptrThreadPlay->Start();

  RTC_LOG(LS_INFO) << __FUNCTION__ << " Started playout capture";
  return 0;
}

int32_t ROSAudioDevice::StopPlayout() {
  {
    rtc::CritScope lock(&_critSect);
    _playing = false;
  }

  if (_ptrThreadPlay) {
    _ptrThreadPlay->Stop();
    _ptrThreadPlay.reset();
  }

  rtc::CritScope lock(&_critSect);

  _playoutFramesLeft = 0;
  delete[] _playoutBuffer;
  _playoutBuffer = NULL;

  RTC_LOG(LS_INFO) << __FUNCTION__ << " Stopped playout capture";
  return 0;
}

bool ROSAudioDevice::Playing() const {
  return _playing;
}

int32_t ROSAudioDevice::StartRecording() {
  if (_recording) {
    return -1;
  }
  _recording = true;

  // Make sure we only create the buffer once.
  _recordingBufferSizeIn10MS =
      _recordingFramesIn10MS * _config.audio_topic_ch * 2;
  if (!_recordingBuffer) {
    _recordingBuffer = new int8_t[_recordingBufferSizeIn10MS];
  }

  ros::NodeHandle nh;
  _sub = nh.subscribe<audio_common_msgs::AudioData>(
      _config.audio_topic_name, 1,
      boost::bind(&ROSAudioDevice::RecROSCallback, this, _1));

  _writtenBufferSize = 0;
  _spinner = new ros::AsyncSpinner(1);
  _spinner->start();

  RTC_LOG(LS_INFO) << __FUNCTION__ << " Started recording";

  return 0;
}

int32_t ROSAudioDevice::StopRecording() {
  {
    rtc::CritScope lock(&_critSect);
    if (!_recording) {
      return -1;
    }
    _recording = false;
  }

  if (_spinner) {
    _spinner->stop();
  }

  rtc::CritScope lock(&_critSect);
  _recordingFramesLeft = 0;
  if (_recordingBuffer) {
    delete[] _recordingBuffer;
    _recordingBuffer = NULL;
  }

  RTC_LOG(LS_INFO) << __FUNCTION__ << " Stopped recording";
  return 0;
}

bool ROSAudioDevice::Recording() const {
  return _recording;
}

int32_t ROSAudioDevice::InitSpeaker() {
  return 0;
}

bool ROSAudioDevice::SpeakerIsInitialized() const {
  return true;
}

int32_t ROSAudioDevice::InitMicrophone() {
  return 0;
}

bool ROSAudioDevice::MicrophoneIsInitialized() const {
  return true;
}

int32_t ROSAudioDevice::SpeakerVolumeIsAvailable(bool& available) {
  return -1;
}

int32_t ROSAudioDevice::SetSpeakerVolume(uint32_t volume) {
  return -1;
}

int32_t ROSAudioDevice::SpeakerVolume(uint32_t& volume) const {
  return -1;
}

int32_t ROSAudioDevice::MaxSpeakerVolume(uint32_t& maxVolume) const {
  return -1;
}

int32_t ROSAudioDevice::MinSpeakerVolume(uint32_t& minVolume) const {
  return -1;
}

int32_t ROSAudioDevice::MicrophoneVolumeIsAvailable(bool& available) {
  return -1;
}

int32_t ROSAudioDevice::SetMicrophoneVolume(uint32_t volume) {
  return -1;
}

int32_t ROSAudioDevice::MicrophoneVolume(uint32_t& volume) const {
  return -1;
}

int32_t ROSAudioDevice::MaxMicrophoneVolume(uint32_t& maxVolume) const {
  return -1;
}

int32_t ROSAudioDevice::MinMicrophoneVolume(uint32_t& minVolume) const {
  return -1;
}

int32_t ROSAudioDevice::SpeakerMuteIsAvailable(bool& available) {
  return -1;
}

int32_t ROSAudioDevice::SetSpeakerMute(bool enable) {
  return -1;
}

int32_t ROSAudioDevice::SpeakerMute(bool& enabled) const {
  return -1;
}

int32_t ROSAudioDevice::MicrophoneMuteIsAvailable(bool& available) {
  return -1;
}

int32_t ROSAudioDevice::SetMicrophoneMute(bool enable) {
  return -1;
}

int32_t ROSAudioDevice::MicrophoneMute(bool& enabled) const {
  return -1;
}

int32_t ROSAudioDevice::StereoPlayoutIsAvailable(bool& available) {
  available = true;
  return 0;
}
int32_t ROSAudioDevice::SetStereoPlayout(bool enable) {
  return 0;
}

int32_t ROSAudioDevice::StereoPlayout(bool& enabled) const {
  enabled = true;
  return 0;
}

int32_t ROSAudioDevice::StereoRecordingIsAvailable(bool& available) {
  available = _config.audio_topic_ch == 2;
  return 0;
}

int32_t ROSAudioDevice::SetStereoRecording(bool enable) {
  return ((_config.audio_topic_ch == 2) == enable) ? 0 : -1;
}

int32_t ROSAudioDevice::StereoRecording(bool& enabled) const {
  enabled = _config.audio_topic_ch == 2;
  return 0;
}

int32_t ROSAudioDevice::PlayoutDelay(uint16_t& delayMS) const {
  return 0;
}

void ROSAudioDevice::AttachAudioBuffer(webrtc::AudioDeviceBuffer* audioBuffer) {
  rtc::CritScope lock(&_critSect);

  _ptrAudioBuffer = audioBuffer;

  // Inform the AudioBuffer about default settings for this implementation.
  // Set all values to zero here since the actual settings will be done by
  // InitPlayout and InitRecording later.
  _ptrAudioBuffer->SetRecordingSampleRate(0);
  _ptrAudioBuffer->SetPlayoutSampleRate(0);
  _ptrAudioBuffer->SetRecordingChannels(0);
  _ptrAudioBuffer->SetPlayoutChannels(0);
}

#if defined(WEBRTC_IOS)
int ROSAudioDevice::GetPlayoutAudioParameters(
    webrtc::AudioParameters* params) const {
  RTC_LOG(INFO) << __FUNCTION__;
  return 0;
}

int ROSAudioDevice::GetRecordAudioParameters(
    webrtc::AudioParameters* params) const {
  RTC_LOG(INFO) << __FUNCTION__;
  return 0;
}
#endif  // WEBRTC_IOS

void ROSAudioDevice::PlayThreadFunc(void* pThis) {
  (static_cast<ROSAudioDevice*>(pThis)->PlayThreadProcess());
}

void ROSAudioDevice::PlayThreadProcess() {
  if (!_playing) {
    return;
  }
  int64_t currentTime = rtc::TimeMillis();
  _critSect.Enter();

  if (_lastCallPlayoutMillis == 0 ||
      currentTime - _lastCallPlayoutMillis >= 5) {
    _critSect.Leave();
    _ptrAudioBuffer->RequestPlayoutData(_playoutFramesIn10MS);
    _critSect.Enter();

    _playoutFramesLeft = _ptrAudioBuffer->GetPlayoutData(_playoutBuffer);
    RTC_DCHECK_EQ(_playoutFramesIn10MS, _playoutFramesLeft);
    _lastCallPlayoutMillis = currentTime;
  }
  _playoutFramesLeft = 0;
  _critSect.Leave();

  int64_t deltaTimeMillis = rtc::TimeMillis() - currentTime;
  if (deltaTimeMillis < 5) {
    webrtc::SleepMs(5 - deltaTimeMillis);
  }
}

bool ROSAudioDevice::RecROSCallback(
    const audio_common_msgs::AudioDataConstPtr& msg) {
  size_t copyedDataSize = 0;
  _critSect.Enter();
  while (_recording && copyedDataSize != msg->data.size()) {
    RTC_LOG(LS_VERBOSE) << "RecROSCallback _recordingBufferSizeIn10MS:"
                        << _recordingBufferSizeIn10MS << " _writtenBufferSize"
                        << _writtenBufferSize << " msg->data.size()"
                        << msg->data.size() << " copyedDataSize"
                        << copyedDataSize;
    if (_recordingBufferSizeIn10MS - _writtenBufferSize <=
        msg->data.size() - copyedDataSize) {
      memcpy(_recordingBuffer + _writtenBufferSize, &msg->data[copyedDataSize],
             _recordingBufferSizeIn10MS - _writtenBufferSize);
      copyedDataSize += _recordingBufferSizeIn10MS - _writtenBufferSize;
      _writtenBufferSize = 0;
      _ptrAudioBuffer->SetRecordedBuffer(_recordingBuffer,
                                         _recordingFramesIn10MS);
      _critSect.Leave();
      _ptrAudioBuffer->DeliverRecordedData();
      webrtc::SleepMs(10);
      _critSect.Enter();
    } else {
      memcpy(_recordingBuffer + _writtenBufferSize, &msg->data[copyedDataSize],
             msg->data.size() - copyedDataSize);
      _writtenBufferSize += msg->data.size() - copyedDataSize;
      copyedDataSize += msg->data.size() - copyedDataSize;
    }
  }
  _critSect.Leave();

  return true;
}
