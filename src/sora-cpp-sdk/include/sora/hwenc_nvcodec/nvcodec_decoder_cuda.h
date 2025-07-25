#ifndef SORA_HWENC_NVCODEC_NVCODEC_DECODER_CUDA_H_
#define SORA_HWENC_NVCODEC_NVCODEC_DECODER_CUDA_H_

#include <cstdint>
#include <memory>
#include <string>

// clang-format off
#include "sora/fix_cuda_noinline_macro_error.h"
// clang-format on

#include "sora/cuda_context.h"

namespace sora {

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

}  // namespace sora

#endif