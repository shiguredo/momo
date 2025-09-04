#ifndef SORA_HWENC_VPL_VPL_VIDEO_DECODER_H_
#define SORA_HWENC_VPL_VPL_VIDEO_DECODER_H_

#include <memory>

// WebRTC
#include <api/video/video_codec_type.h>
#include <api/video_codecs/video_decoder.h>

#include "sora/vpl_session.h"

namespace sora {

class VplVideoDecoder : public webrtc::VideoDecoder {
 public:
  static bool IsSupported(std::shared_ptr<VplSession> session,
                          webrtc::VideoCodecType codec);
  static std::unique_ptr<VplVideoDecoder> Create(
      std::shared_ptr<VplSession> session,
      webrtc::VideoCodecType codec);
};

}  // namespace sora

#endif
