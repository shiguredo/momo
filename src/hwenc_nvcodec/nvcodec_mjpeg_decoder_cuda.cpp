#include "nvcodec_mjpeg_decoder_cuda.h"

// NvCodec
#include <NvDecoder/NvDecoder.h>

#include "cuda/cuda_context_cuda.h"

NvCodecMjpegDecoderCuda::NvCodecMjpegDecoderCuda(
    std::shared_ptr<CudaContext> ctx)
    : impl_(new NvDecoder(GetCudaContext(ctx),
                          false,
                          cudaVideoCodec_JPEG,
                          false,
                          true,
                          nullptr,
                          nullptr,
                          3840,
                          2160)) {}

static NvDecoder* GetDecoder(std::shared_ptr<void> impl) {
  return std::static_pointer_cast<NvDecoder>(impl).get();
}

int NvCodecMjpegDecoderCuda::Decode(const uint8_t* pData, int nSize) {
  return GetDecoder(impl_)->Decode(pData, nSize);
}

uint8_t* NvCodecMjpegDecoderCuda::GetFrame() {
  return GetDecoder(impl_)->GetFrame();
}

uint8_t* NvCodecMjpegDecoderCuda::GetLockedFrame() {
  return GetDecoder(impl_)->GetLockedFrame();
}

void NvCodecMjpegDecoderCuda::UnlockFrame(uint8_t* pFrame) {
  return GetDecoder(impl_)->UnlockFrame(&pFrame);
}