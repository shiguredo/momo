#ifndef SORA_HWENC_NVCODEC_NVCODEC_VIDEO_ENCODER_CUDA_H_
#define SORA_HWENC_NVCODEC_NVCODEC_VIDEO_ENCODER_CUDA_H_

// CUDA と WebRTC のヘッダを混ぜてコンパイルすると大量のエラーが発生したので、
// CUDA の処理だけさせる単純な CUDA ファイルを用意する

#include <memory>

// NvCodec
#include <NvEncoder/NvEncoder.h>

#include "sora/cuda_context.h"

class NvEncoder;

namespace sora {

class NvCodecVideoEncoderCudaImpl;

class NvCodecVideoEncoderCuda {
 public:
  NvCodecVideoEncoderCuda(std::shared_ptr<CudaContext> ctx);
  ~NvCodecVideoEncoderCuda();

  void Copy(NvEncoder* nv_encoder, const void* ptr, int width, int height);
  // 念のため <memory> も include せずポインタを利用する
  NvEncoder* CreateNvEncoder(int width, int height, bool is_nv12);

 private:
  NvCodecVideoEncoderCudaImpl* impl_;
};

}  // namespace sora

#endif
