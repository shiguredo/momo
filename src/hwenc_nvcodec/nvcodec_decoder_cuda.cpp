#include "fix_cuda_noinline_macro_error.h"

#include "nvcodec_decoder_cuda.h"

// NvCodec
#include <NvDecoder/NvDecoder.h>

#include "cuda/cuda_context_cuda.h"

static cudaVideoCodec ToCudaVideoCodec(CudaVideoCodec codec) {
  return codec == CudaVideoCodec::H264  ? cudaVideoCodec_H264
         : codec == CudaVideoCodec::VP8 ? cudaVideoCodec_VP8
         : codec == CudaVideoCodec::VP9 ? cudaVideoCodec_VP9
                                        : cudaVideoCodec_JPEG;
}

#define CUDA_DRVAPI_CALL(call)                                      \
  do {                                                              \
    CUresult err__ = call;                                          \
    if (err__ != CUDA_SUCCESS) {                                    \
      const char* szErrName = NULL;                                 \
      dyn::cuGetErrorName(err__, &szErrName);                       \
      std::ostringstream errorLog;                                  \
      errorLog << "CUDA driver API error " << szErrName;            \
      throw NVDECException::makeNVDECException(                     \
          errorLog.str(), err__, __FUNCTION__, __FILE__, __LINE__); \
    }                                                               \
  } while (0)

NvCodecDecoderCuda::NvCodecDecoderCuda(std::shared_ptr<CudaContext> ctx,
                                       CudaVideoCodec codec)
    : impl_(new NvDecoder(GetCudaContext(ctx),
                          false,
                          ToCudaVideoCodec(codec),
                          false,
                          true,
                          nullptr,
                          nullptr,
                          3840,
                          3840)) {
  // このコーデックでデコード可能かどうかを調べる
  CUVIDDECODECAPS decodecaps;
  memset(&decodecaps, 0, sizeof(decodecaps));

  decodecaps.eCodecType = ToCudaVideoCodec(codec);
  decodecaps.eChromaFormat = cudaVideoChromaFormat_420;
  decodecaps.nBitDepthMinus8 = 0;

  CUDA_DRVAPI_CALL(dyn::cuCtxPushCurrent(GetCudaContext(ctx)));
  NVDEC_API_CALL(dyn::cuvidGetDecoderCaps(&decodecaps));
  CUDA_DRVAPI_CALL(dyn::cuCtxPopCurrent(nullptr));

  if (!decodecaps.bIsSupported) {
    throw NVDECException::makeNVDECException(
        "Specified video codec not supported", CUDA_ERROR_NOT_SUPPORTED,
        __FUNCTION__, __FILE__, __LINE__);
  }
}

static NvDecoder* GetDecoder(std::shared_ptr<void> impl) {
  return std::static_pointer_cast<NvDecoder>(impl).get();
}

int NvCodecDecoderCuda::Decode(const uint8_t* pData, int nSize) {
  return GetDecoder(impl_)->Decode(pData, nSize);
}

uint8_t* NvCodecDecoderCuda::GetFrame() {
  return GetDecoder(impl_)->GetFrame();
}

uint8_t* NvCodecDecoderCuda::GetLockedFrame() {
  return GetDecoder(impl_)->GetLockedFrame();
}

void NvCodecDecoderCuda::UnlockFrame(uint8_t* pFrame) {
  GetDecoder(impl_)->UnlockFrame(&pFrame);
}

std::string NvCodecDecoderCuda::GetVideoInfo() const {
  return GetDecoder(impl_)->GetVideoInfo();
}
int NvCodecDecoderCuda::GetWidth() const {
  return GetDecoder(impl_)->GetWidth();
}
int NvCodecDecoderCuda::GetHeight() const {
  return GetDecoder(impl_)->GetHeight();
}
int NvCodecDecoderCuda::GetDeviceFramePitch() const {
  return GetDecoder(impl_)->GetDeviceFramePitch();
}

int NvCodecDecoderCuda::setReconfigParams() {
  return GetDecoder(impl_)->setReconfigParams(nullptr, nullptr);
}