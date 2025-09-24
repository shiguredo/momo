#ifndef SORA_HWENC_V4L2_V4L2_VIDEO_CODEC_H_
#define SORA_HWENC_V4L2_V4L2_VIDEO_CODEC_H_

#include <memory>

#include "sora/sora_video_codec.h"

namespace sora {

VideoCodecCapability::Engine GetV4L2VideoCodecCapability();

}  // namespace sora

#endif
