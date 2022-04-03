#ifndef NVCODEC_DECODER_CUDA_H_
#define NVCODEC_DECODER_CUDA_H_

#include <memory>
#include <string>

#include "cuda/cuda_context.h"

class NvCodecDecoderCuda {
 public:
  NvCodecDecoderCuda(std::shared_ptr<CudaContext> ctx, CudaVideoCodec codec);
  int Decode(const uint8_t* pData, int nSize);
  uint8_t* GetFrame();
  uint8_t* GetLockedFrame();
  void UnlockFrame(uint8_t* pFrame);

  std::string GetVideoInfo() const;
  int GetWidth() const;
  int GetHeight() const;
  int GetDeviceFramePitch() const;

  int setReconfigParams();

 private:
  std::shared_ptr<void> impl_;
};

#endif