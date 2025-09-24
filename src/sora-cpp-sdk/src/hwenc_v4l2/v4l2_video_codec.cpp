#include "sora/hwenc_v4l2/v4l2_video_codec.h"
#include <memory>
#include <string>

// WebRTC
#include <api/video/video_codec_type.h>

#include "sora/hwenc_v4l2/v4l2_h264_decoder.h"
#include "sora/hwenc_v4l2/v4l2_h264_encoder.h"
#include "sora/sora_video_codec.h"

namespace sora {

VideoCodecCapability::Engine GetV4L2VideoCodecCapability() {
  VideoCodecCapability::Engine engine(VideoCodecImplementation::kRaspiV4L2M2M);

  auto add = [&engine](webrtc::VideoCodecType type) {
    engine.codecs.emplace_back(type, V4L2H264Encoder::IsSupported(type),
                               V4L2H264Decoder::IsSupported(type));
  };
  add(webrtc::kVideoCodecVP8);
  add(webrtc::kVideoCodecVP9);
  add(webrtc::kVideoCodecH264);
  add(webrtc::kVideoCodecH265);
  add(webrtc::kVideoCodecAV1);
  return engine;
}

}  // namespace sora
