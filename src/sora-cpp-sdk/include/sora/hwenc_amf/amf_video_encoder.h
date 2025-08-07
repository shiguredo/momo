#ifndef SORA_HWENC_AMF_AMF_VIDEO_ENCODER_H_
#define SORA_HWENC_AMF_AMF_VIDEO_ENCODER_H_

#include <memory>

// WebRTC
#include <api/video/video_codec_type.h>
#include <api/video_codecs/video_encoder.h>

#include "sora/amf_context.h"

namespace sora {

class AMFVideoEncoder : public webrtc::VideoEncoder {
 public:
  static bool IsSupported(std::shared_ptr<AMFContext> amf_context,
                          webrtc::VideoCodecType codec);
  static std::unique_ptr<AMFVideoEncoder> Create(
      std::shared_ptr<AMFContext> amf_context,
      webrtc::VideoCodecType codec);
};

}  // namespace sora

#endif
