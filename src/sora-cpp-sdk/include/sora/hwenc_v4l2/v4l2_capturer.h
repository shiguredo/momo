#ifndef SORA_HWENC_V4L2_V4L2_CAPTURER_H_
#define SORA_HWENC_V4L2_V4L2_CAPTURER_H_

#include "sora/v4l2/v4l2_video_capturer.h"

namespace sora {

class V4L2Capturer : public V4L2VideoCapturer {
 public:
  static webrtc::scoped_refptr<V4L2Capturer> Create(
      V4L2VideoCapturerConfig config);
  V4L2Capturer(const V4L2VideoCapturerConfig& config);

  void OnCaptured(uint8_t* data, uint32_t bytesused) override;
};

}  // namespace sora

#endif  // NVCODEC_V4L2_CAPTURER_H_
