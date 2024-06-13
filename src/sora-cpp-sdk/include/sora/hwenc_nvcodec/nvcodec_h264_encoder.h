#ifndef SORA_HWENC_NVCODEC_NVCODEC_H264_ENCODER_H_
#define SORA_HWENC_NVCODEC_NVCODEC_H264_ENCODER_H_

#include <memory>

// WebRTC
#include <api/video_codecs/video_encoder.h>
#include <media/base/codec.h>

#include "sora/cuda_context.h"

namespace sora {

class NvCodecH264Encoder : public webrtc::VideoEncoder {
 public:
  static bool IsSupported(std::shared_ptr<CudaContext> cuda_context);
  static std::unique_ptr<NvCodecH264Encoder> Create(
      const cricket::VideoCodec& codec,
      std::shared_ptr<CudaContext> cuda_context);
};

}  // namespace sora

#endif
