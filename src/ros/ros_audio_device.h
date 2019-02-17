/*
 *  Copyright (c) 2012 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef ROS_AUDIO_DEVICE_H_
#define ROS_AUDIO_DEVICE_H_

#include <memory>

#include "modules/audio_device/audio_device_generic.h"
#include "rtc_base/platform_thread.h"
#include "rtc_base/criticalsection.h"
#include "rtc_base/system/file_wrapper.h"
#include "rtc_base/timeutils.h"

#include "ros/ros.h"
#include "audio_common_msgs/AudioData.h"

#include "connection_settings.h"

class ROSAudioDevice : public webrtc::AudioDeviceGeneric
{
public:
  ROSAudioDevice(ConnectionSettings conn_settings);
  ~ROSAudioDevice() override;

  // Retrieve the currently utilized audio layer
  int32_t ActiveAudioLayer(
      webrtc::AudioDeviceModule::AudioLayer &audioLayer) const override;

  // Main initializaton and termination
  webrtc::AudioDeviceGeneric::InitStatus Init() override;
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
  int32_t SetPlayoutDevice(
      webrtc::AudioDeviceModule::WindowsDeviceType device) override;
  int32_t SetRecordingDevice(uint16_t index) override;
  int32_t SetRecordingDevice(
      webrtc::AudioDeviceModule::WindowsDeviceType device) override;

  // Audio transport initialization
  int32_t PlayoutIsAvailable(bool &available) override;
  int32_t InitPlayout() override;
  bool PlayoutIsInitialized() const override;
  int32_t RecordingIsAvailable(bool &available) override;
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
  int32_t SpeakerVolumeIsAvailable(bool &available) override;
  int32_t SetSpeakerVolume(uint32_t volume) override;
  int32_t SpeakerVolume(uint32_t &volume) const override;
  int32_t MaxSpeakerVolume(uint32_t &maxVolume) const override;
  int32_t MinSpeakerVolume(uint32_t &minVolume) const override;

  // Microphone volume controls
  int32_t MicrophoneVolumeIsAvailable(bool &available) override;
  int32_t SetMicrophoneVolume(uint32_t volume) override;
  int32_t MicrophoneVolume(uint32_t &volume) const override;
  int32_t MaxMicrophoneVolume(uint32_t &maxVolume) const override;
  int32_t MinMicrophoneVolume(uint32_t &minVolume) const override;

  // Speaker mute control
  int32_t SpeakerMuteIsAvailable(bool &available) override;
  int32_t SetSpeakerMute(bool enable) override;
  int32_t SpeakerMute(bool &enabled) const override;

  // Microphone mute control
  int32_t MicrophoneMuteIsAvailable(bool &available) override;
  int32_t SetMicrophoneMute(bool enable) override;
  int32_t MicrophoneMute(bool &enabled) const override;

  // Stereo support
  int32_t StereoPlayoutIsAvailable(bool &available) override;
  int32_t SetStereoPlayout(bool enable) override;
  int32_t StereoPlayout(bool &enabled) const override;
  int32_t StereoRecordingIsAvailable(bool &available) override;
  int32_t SetStereoRecording(bool enable) override;
  int32_t StereoRecording(bool &enabled) const override;

  // Delay information and control
  int32_t PlayoutDelay(uint16_t &delayMS) const override;

  void AttachAudioBuffer(webrtc::AudioDeviceBuffer *audioBuffer) override;

#if defined(WEBRTC_IOS)
  int GetPlayoutAudioParameters(webrtc::AudioParameters* params) const override;
  int GetRecordAudioParameters(webrtc::AudioParameters* params) const override;
#endif // WEBRTC_IOS

private:
  static bool PlayThreadFunc(void *);
  bool RecROSCallback(const audio_common_msgs::AudioDataConstPtr &msg);
  bool PlayThreadProcess();

  ConnectionSettings _conn_settings;

  int32_t _playout_index;
  int32_t _record_index;
  webrtc::AudioDeviceBuffer *_ptrAudioBuffer;
  int8_t *_recordingBuffer; // In bytes.
  int8_t *_playoutBuffer;   // In bytes.
  uint32_t _recordingFramesLeft;
  uint32_t _playoutFramesLeft;
  rtc::CriticalSection _critSect;

  size_t _recordingBufferSizeIn10MS;
  size_t _recordingFramesIn10MS;
  size_t _writtenBufferSize;
  size_t _playoutFramesIn10MS;

  ros::AsyncSpinner* _spinner;
  ros::Subscriber _sub;

  // TODO(pbos): Make plain members instead of pointers and stop resetting them.
  std::unique_ptr<rtc::PlatformThread> _ptrThreadPlay;

  bool _playing;
  bool _recording;
  int64_t _lastCallPlayoutMillis;
  int64_t _lastCallRecordMillis;
};

#endif