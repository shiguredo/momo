#ifndef MOMO_HWENC_AMF_AMF_VIDEO_ENCODER_H_
#define MOMO_HWENC_AMF_AMF_VIDEO_ENCODER_H_

#include <memory>

// WebRTC
#include <api/video/video_codec_type.h>
#include <api/video_codecs/video_encoder.h>

#include "amf_context.h"

namespace momo {

class AMFVideoEncoder : public webrtc::VideoEncoder {
 public:
  static bool IsSupported(std::shared_ptr<AMFContext> amf_context,
                          webrtc::VideoCodecType codec);
  static std::unique_ptr<AMFVideoEncoder> Create(
      std::shared_ptr<AMFContext> amf_context,
      webrtc::VideoCodecType codec);
};

}  // namespace momo

#endif  // MOMO_HWENC_AMF_AMF_VIDEO_ENCODER_H_