#ifndef SORA_HWENC_VPL_VPL_VIDEO_ENCODER_H_
#define SORA_HWENC_VPL_VPL_VIDEO_ENCODER_H_

#include <memory>

// WebRTC
#include <api/video/video_codec_type.h>
#include <api/video_codecs/video_encoder.h>

#include "sora/vpl_session.h"

namespace sora {

class VplVideoEncoder : public webrtc::VideoEncoder {
 public:
  static bool IsSupported(std::shared_ptr<VplSession> session,
                          webrtc::VideoCodecType codec);
  static std::unique_ptr<VplVideoEncoder> Create(
      std::shared_ptr<VplSession> session,
      webrtc::VideoCodecType codec);

  // DMABUF サポート
  virtual std::vector<int> GetDmaBufFds() const { return {}; }
  virtual bool EnableDmaBufMode() { return false; }
};

}  // namespace sora

#endif
