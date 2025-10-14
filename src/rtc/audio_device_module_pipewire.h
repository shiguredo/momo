#ifndef RTC_AUDIO_DEVICE_MODULE_PIPEWIRE_H_
#define RTC_AUDIO_DEVICE_MODULE_PIPEWIRE_H_

#include <api/audio/audio_device.h>
#include <api/scoped_refptr.h>

namespace webrtc {

// PipeWire 用の AudioDeviceModule を作成する
webrtc::scoped_refptr<AudioDeviceModule> CreatePipeWireAudioDeviceModule();

}  // namespace webrtc

#endif  // RTC_AUDIO_DEVICE_MODULE_PIPEWIRE_H_
