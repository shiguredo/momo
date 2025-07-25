#ifndef SORA_HWENC_NVCODEC_NVCODEC_VIDEO_ENCODER_H_
#define SORA_HWENC_NVCODEC_NVCODEC_VIDEO_ENCODER_H_

#include <memory>

// WebRTC
#include <api/video_codecs/video_encoder.h>

#include "sora/cuda_context.h"

namespace sora {

class NvCodecVideoEncoder : public webrtc::VideoEncoder {
 public:
  static bool IsSupported(std::shared_ptr<CudaContext> cuda_context,
                          CudaVideoCodec codec);
  static std::unique_ptr<NvCodecVideoEncoder> Create(
      std::shared_ptr<CudaContext> cuda_context,
      CudaVideoCodec codec);
};

}  // namespace sora

#endif
