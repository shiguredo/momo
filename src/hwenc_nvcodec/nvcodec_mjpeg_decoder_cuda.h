#ifndef NVCODEC_MJPEG_DECODER_CUDA_H_
#define NVCODEC_MJPEG_DECODER_CUDA_H_

#include <memory>

#include "cuda/cuda_context.h"

class NvCodecMjpegDecoderCuda {
 public:
  NvCodecMjpegDecoderCuda(std::shared_ptr<CudaContext> ctx);
  int Decode(const uint8_t* pData, int nSize);
  uint8_t* GetFrame();
  uint8_t* GetLockedFrame();
  void UnlockFrame(uint8_t* pFrame);

 private:
  std::shared_ptr<void> impl_;
};

#endif  // NVCODEC_MJPEG_DECODER_CUDA_H_