#ifndef MOMO_HWENC_AMF_AMF_VIDEO_DECODER_H_
#define MOMO_HWENC_AMF_AMF_VIDEO_DECODER_H_

#include <memory>

// WebRTC
#include <api/video/video_codec_type.h>
#include <api/video_codecs/video_decoder.h>

#include "amf_context.h"

namespace momo {

class AMFVideoDecoder : public webrtc::VideoDecoder {
 public:
  static bool IsSupported(std::shared_ptr<AMFContext> amf_context,
                          webrtc::VideoCodecType codec);
  static std::unique_ptr<AMFVideoDecoder> Create(
      std::shared_ptr<AMFContext> amf_context,
      webrtc::VideoCodecType codec);
};

}  // namespace momo

#endif  // MOMO_HWENC_AMF_AMF_VIDEO_DECODER_H_