#include "device_info.h"

// WebRTC
#include <api/audio/create_audio_device_module.h>
#include <api/environment/environment_factory.h>
#include <rtc_base/logging.h>

#if defined(__APPLE__)
#include "mac_helper/mac_capturer.h"
#endif

std::vector<VideoDeviceInfo> GetVideoDeviceInfos() {
#if defined(__APPLE__)
  return MacCapturer::GetVideoDeviceInfos();
#else
  // 他のプラットフォームでは未実装
  return std::vector<VideoDeviceInfo>();
#endif
}

std::vector<AudioDeviceInfo> GetAudioDeviceInfos(
    webrtc::AudioDeviceModule::AudioLayer layer,
    bool is_input) {
  std::vector<AudioDeviceInfo> device_infos;

  auto adm =
      webrtc::CreateAudioDeviceModule(webrtc::CreateEnvironment(), layer);
  if (!adm) {
    RTC_LOG(LS_ERROR) << "Failed to create AudioDeviceModule";
    return device_infos;
  }

  if (adm->Init() != 0) {
    RTC_LOG(LS_ERROR) << "AudioDeviceModule::Init failed";
    return device_infos;
  }

  device_infos = GetAudioDeviceInfos(adm, is_input);

  adm->Terminate();

  return device_infos;
}
std::vector<AudioDeviceInfo> GetAudioDeviceInfos(
    webrtc::scoped_refptr<webrtc::AudioDeviceModule> adm,
    bool is_input) {
  std::vector<AudioDeviceInfo> device_infos;

  int16_t device_count =
      is_input ? adm->RecordingDevices() : adm->PlayoutDevices();
  for (int16_t i = 0; i < device_count; ++i) {
    char name[webrtc::kAdmMaxDeviceNameSize] = {0};
    char guid[webrtc::kAdmMaxGuidSize] = {0};
    int32_t result = is_input ? adm->RecordingDeviceName(i, name, guid)
                              : adm->PlayoutDeviceName(i, name, guid);
    if (result != 0) {
      continue;
    }
    AudioDeviceInfo info;
    info.index = i;
    info.name = name;
    info.guid = guid;
    info.is_input = is_input;
    device_infos.push_back(info);
  }

  return device_infos;
}
