#ifndef SORA_V4L2_VPL_CAPTURER_H_
#define SORA_V4L2_VPL_CAPTURER_H_

#include <memory>
#include <vector>

// WebRTC
#include <api/scoped_refptr.h>

#include "sora/hwenc_vpl/vpl_video_encoder.h"
#include "sora/v4l2/v4l2_video_capturer.h"
#include "sora/vpl_session.h"

namespace sora {

// V4L2 と VPL を DMABUF で接続するヘルパークラス
class V4L2VplCapturer {
 public:
  // V4L2 キャプチャと VPL エンコーダーを DMABUF で接続
  static webrtc::scoped_refptr<V4L2VideoCapturer> CreateWithVplEncoder(
      const V4L2VideoCapturerConfig& config,
      std::shared_ptr<VplVideoEncoder> encoder);

  // VPL エンコーダーから DMABUF fd を取得
  static std::vector<int> GetDmaBufFdsFromEncoder(
      std::shared_ptr<VplVideoEncoder> encoder);
};

}  // namespace sora

#endif  // SORA_V4L2_VPL_CAPTURER_H_