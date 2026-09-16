#ifndef SORA_AUDIO_DEVICE_HELPER_H_
#define SORA_AUDIO_DEVICE_HELPER_H_

#include <string>
#include <tuple>
#include <vector>

namespace sora {

// デバイス名または GUID と一致するデバイスのインデックスを探す
//
// Args:
//   device_name: 検索するデバイス名または GUID
//   devices: (名前, GUID) のタプル一覧
//
// Returns:
//   一致したデバイスのインデックス。一致しない場合は -1
int FindAudioDeviceIndex(
    const std::string& device_name,
    const std::vector<std::tuple<std::string, std::string> >& devices);

}  // namespace sora

#endif
