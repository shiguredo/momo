#ifndef V4L2_CAPTURER_H_
#define V4L2_CAPTURER_H_

#include "sora/v4l2/v4l2_video_capturer.h"

class V4L2Capturer : public sora::V4L2VideoCapturer {
 public:
  static webrtc::scoped_refptr<V4L2Capturer> Create(
      sora::V4L2VideoCapturerConfig config);
  V4L2Capturer(const sora::V4L2VideoCapturerConfig& config);

  void OnCaptured(uint8_t* data, uint32_t bytesused) override;
};

#endif  // NVCODEC_V4L2_CAPTURER_H_
