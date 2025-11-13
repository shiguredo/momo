#ifndef MOMO_DEVICE_INFO_H_
#define MOMO_DEVICE_INFO_H_

#include <string>
#include <vector>

// WebRTC
#include <api/audio/audio_device.h>
#include <api/scoped_refptr.h>

struct AudioDeviceInfo {
  int index;
  std::string name;
  std::string guid;
  bool is_input;
};

struct VideoDeviceFormat {
  int width;
  int height;
  std::vector<int> framerates;  // FPS のリスト
};

struct VideoDeviceInfo {
  int index;
  std::string name;
  std::vector<VideoDeviceFormat> formats;
};

std::vector<VideoDeviceInfo> GetVideoDeviceInfos();
std::vector<AudioDeviceInfo> GetAudioDeviceInfos(
    webrtc::AudioDeviceModule::AudioLayer layer,
    bool is_input);
std::vector<AudioDeviceInfo> GetAudioDeviceInfos(
    webrtc::scoped_refptr<webrtc::AudioDeviceModule> adm,
    bool is_input);

#endif
