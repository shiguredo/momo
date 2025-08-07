#ifndef SORA_HWENC_AMF_AMF_VIDEO_CODEC_H_
#define SORA_HWENC_AMF_AMF_VIDEO_CODEC_H_

#include <memory>

#include "sora/amf_context.h"
#include "sora/sora_video_codec.h"

namespace sora {

VideoCodecCapability::Engine GetAMFVideoCodecCapability(
    std::shared_ptr<AMFContext> context);

}  // namespace sora

#endif
