#include "amf_video_decoder.h"

namespace momo {

bool AMFVideoDecoder::IsSupported(std::shared_ptr<AMFContext> amf_context,
                                  webrtc::VideoCodecType codec) {
  // デコーダのサポートは今後実装予定
  return false;
}

std::unique_ptr<AMFVideoDecoder> AMFVideoDecoder::Create(
    std::shared_ptr<AMFContext> amf_context,
    webrtc::VideoCodecType codec) {
  // デコーダの実装は今後実装予定
  return nullptr;
}

}  // namespace momo