#include "sora/fix_cuda_noinline_macro_error.h"

#include "nvcodec_video_encoder_cuda.h"

#include <cstddef>
#include <memory>

// CUDA
#include <cuda.h>

// NvCodec
#include <NvEncoder/../../Utils/NvCodecUtils.h>
#include <NvEncoder/NvEncoder.h>
#include <NvEncoder/NvEncoderCuda.h>
#include <nvEncodeAPI.h>

#include "../cuda_context_cuda.h"
#include "sora/cuda_context.h"
#include "sora/dyn/cuda.h"

namespace sora {

class NvCodecVideoEncoderCudaImpl {
 public:
  NvCodecVideoEncoderCudaImpl(std::shared_ptr<CudaContext> ctx);
  ~NvCodecVideoEncoderCudaImpl();

  void Copy(NvEncoder* nv_encoder, const void* ptr, int width, int height);
  NvEncoder* CreateNvEncoder(int width, int height, bool is_nv12);

 private:
  std::shared_ptr<CudaContext> cuda_context_;
};

NvCodecVideoEncoderCuda::NvCodecVideoEncoderCuda(
    std::shared_ptr<CudaContext> ctx)
    : impl_(new NvCodecVideoEncoderCudaImpl(ctx)) {}
NvCodecVideoEncoderCuda::~NvCodecVideoEncoderCuda() {
  delete impl_;
}

void NvCodecVideoEncoderCuda::Copy(NvEncoder* nv_encoder,
                                   const void* ptr,
                                   int width,
                                   int height) {
  impl_->Copy(nv_encoder, ptr, width, height);
}
NvEncoder* NvCodecVideoEncoderCuda::CreateNvEncoder(int width,
                                                    int height,
                                                    bool is_nv12) {
  return impl_->CreateNvEncoder(width, height, is_nv12);
}

NvCodecVideoEncoderCudaImpl::NvCodecVideoEncoderCudaImpl(
    std::shared_ptr<CudaContext> ctx) {
  cuda_context_ = ctx;
}
NvCodecVideoEncoderCudaImpl::~NvCodecVideoEncoderCudaImpl() {}
void NvCodecVideoEncoderCudaImpl::Copy(NvEncoder* nv_encoder,
                                       const void* ptr,
                                       int width,
                                       int height) {
  const NvEncInputFrame* input_frame = nv_encoder->GetNextInputFrame();
  CUcontext context = GetCudaContext(cuda_context_);
  NvEncoderCuda::CopyToDeviceFrame(
      context, (void*)ptr, 0, (CUdeviceptr)input_frame->inputPtr,
      (int)input_frame->pitch, width, height, CU_MEMORYTYPE_HOST,
      input_frame->bufferFormat, input_frame->chromaOffsets,
      input_frame->numChromaPlanes);
}
NvEncoder* NvCodecVideoEncoderCudaImpl::CreateNvEncoder(int width,
                                                        int height,
                                                        bool is_nv12) {
  NV_ENC_BUFFER_FORMAT nvenc_format =
      is_nv12 ? NV_ENC_BUFFER_FORMAT_NV12 : NV_ENC_BUFFER_FORMAT_IYUV;
  CUcontext context = GetCudaContext(cuda_context_);
  return new NvEncoderCuda(context, width, height, nvenc_format);
}

}  // namespace sora
