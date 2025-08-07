#ifndef SORA_HWENC_AMF_AMF_VIDEO_DECODER_H_
#define SORA_HWENC_AMF_AMF_VIDEO_DECODER_H_

#include <memory>

// WebRTC
#include <api/video/video_codec_type.h>
#include <api/video_codecs/video_decoder.h>

#include "sora/amf_context.h"

namespace sora {

class AMFVideoDecoder : public webrtc::VideoDecoder {
 public:
  static std::unique_ptr<AMFVideoDecoder> Create(
      std::shared_ptr<AMFContext> context,
      webrtc::VideoCodecType codec);
  static bool IsSupported(std::shared_ptr<AMFContext> context,
                          webrtc::VideoCodecType codec);
};

}  // namespace sora

#endif
