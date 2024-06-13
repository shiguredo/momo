#include "sora/fix_cuda_noinline_macro_error.h"

#include "nvcodec_h264_encoder_cuda.h"

#include <iostream>

// NvCodec
#include <NvDecoder/NvDecoder.h>
#include <NvEncoder/NvEncoderCuda.h>

#include "../cuda_context_cuda.h"
#include "sora/dyn/cuda.h"

namespace sora {

class NvCodecH264EncoderCudaImpl {
 public:
  NvCodecH264EncoderCudaImpl(std::shared_ptr<CudaContext> ctx);
  ~NvCodecH264EncoderCudaImpl();

  void Copy(NvEncoder* nv_encoder, const void* ptr, int width, int height);
  NvEncoder* CreateNvEncoder(int width, int height, bool is_nv12);

 private:
  std::shared_ptr<CudaContext> cuda_context_;
};

NvCodecH264EncoderCuda::NvCodecH264EncoderCuda(std::shared_ptr<CudaContext> ctx)
    : impl_(new NvCodecH264EncoderCudaImpl(ctx)) {}
NvCodecH264EncoderCuda::~NvCodecH264EncoderCuda() {
  delete impl_;
}

void NvCodecH264EncoderCuda::Copy(NvEncoder* nv_encoder,
                                  const void* ptr,
                                  int width,
                                  int height) {
  impl_->Copy(nv_encoder, ptr, width, height);
}
NvEncoder* NvCodecH264EncoderCuda::CreateNvEncoder(int width,
                                                   int height,
                                                   bool is_nv12) {
  return impl_->CreateNvEncoder(width, height, is_nv12);
}

void ShowEncoderCapability() {
  int nGpu = 0;
  ck(dyn::cuDeviceGetCount(&nGpu));
  if (nGpu == 0) {
    std::cerr << "CUDA Device not found" << std::endl;
    exit(1);
  }
  std::cout << "Encoder Capability" << std::endl;
  for (int iGpu = 0; iGpu < nGpu; iGpu++) {
    CUdevice cuDevice = 0;
    ck(dyn::cuDeviceGet(&cuDevice, iGpu));
    char szDeviceName[80];
    ck(dyn::cuDeviceGetName(szDeviceName, sizeof(szDeviceName), cuDevice));
    CUcontext cuContext = NULL;
    ck(dyn::cuCtxCreate(&cuContext, 0, cuDevice));
    NvEncoderCuda enc(cuContext, 1280, 720, NV_ENC_BUFFER_FORMAT_NV12);

    std::cout << "GPU " << iGpu << " - " << szDeviceName << std::endl
              << std::endl;
    std::cout
        << "\tH264:\t\t"
        << "  "
        << (enc.GetCapabilityValue(NV_ENC_CODEC_H264_GUID,
                                   NV_ENC_CAPS_SUPPORTED_RATECONTROL_MODES)
                ? "yes"
                : "no")
        << std::endl
        << "\tH264_444:\t"
        << "  "
        << (enc.GetCapabilityValue(NV_ENC_CODEC_H264_GUID,
                                   NV_ENC_CAPS_SUPPORT_YUV444_ENCODE)
                ? "yes"
                : "no")
        << std::endl
        << "\tH264_ME:\t"
        << "  "
        << (enc.GetCapabilityValue(NV_ENC_CODEC_H264_GUID,
                                   NV_ENC_CAPS_SUPPORT_MEONLY_MODE)
                ? "yes"
                : "no")
        << std::endl
        << "\tH264_WxH:\t"
        << "  "
        << (enc.GetCapabilityValue(NV_ENC_CODEC_H264_GUID,
                                   NV_ENC_CAPS_WIDTH_MAX))
        << "*"
        << (enc.GetCapabilityValue(NV_ENC_CODEC_H264_GUID,
                                   NV_ENC_CAPS_HEIGHT_MAX))
        << std::endl
        << "\tHEVC:\t\t"
        << "  "
        << (enc.GetCapabilityValue(NV_ENC_CODEC_HEVC_GUID,
                                   NV_ENC_CAPS_SUPPORTED_RATECONTROL_MODES)
                ? "yes"
                : "no")
        << std::endl
        << "\tHEVC_Main10:\t"
        << "  "
        << (enc.GetCapabilityValue(NV_ENC_CODEC_HEVC_GUID,
                                   NV_ENC_CAPS_SUPPORT_10BIT_ENCODE)
                ? "yes"
                : "no")
        << std::endl
        << "\tHEVC_Lossless:\t"
        << "  "
        << (enc.GetCapabilityValue(NV_ENC_CODEC_HEVC_GUID,
                                   NV_ENC_CAPS_SUPPORT_LOSSLESS_ENCODE)
                ? "yes"
                : "no")
        << std::endl
        << "\tHEVC_SAO:\t"
        << "  "
        << (enc.GetCapabilityValue(NV_ENC_CODEC_HEVC_GUID,
                                   NV_ENC_CAPS_SUPPORT_SAO)
                ? "yes"
                : "no")
        << std::endl
        << "\tHEVC_444:\t"
        << "  "
        << (enc.GetCapabilityValue(NV_ENC_CODEC_HEVC_GUID,
                                   NV_ENC_CAPS_SUPPORT_YUV444_ENCODE)
                ? "yes"
                : "no")
        << std::endl
        << "\tHEVC_ME:\t"
        << "  "
        << (enc.GetCapabilityValue(NV_ENC_CODEC_HEVC_GUID,
                                   NV_ENC_CAPS_SUPPORT_MEONLY_MODE)
                ? "yes"
                : "no")
        << std::endl
        << "\tHEVC_WxH:\t"
        << "  "
        << (enc.GetCapabilityValue(NV_ENC_CODEC_HEVC_GUID,
                                   NV_ENC_CAPS_WIDTH_MAX))
        << "*"
        << (enc.GetCapabilityValue(NV_ENC_CODEC_HEVC_GUID,
                                   NV_ENC_CAPS_HEIGHT_MAX))
        << std::endl;

    std::cout << std::endl;

    enc.DestroyEncoder();
    ck(dyn::cuCtxDestroy(cuContext));
  }
}

NvCodecH264EncoderCudaImpl::NvCodecH264EncoderCudaImpl(
    std::shared_ptr<CudaContext> ctx) {
  ShowEncoderCapability();
  cuda_context_ = ctx;
}
NvCodecH264EncoderCudaImpl::~NvCodecH264EncoderCudaImpl() {}
void NvCodecH264EncoderCudaImpl::Copy(NvEncoder* nv_encoder,
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
NvEncoder* NvCodecH264EncoderCudaImpl::CreateNvEncoder(int width,
                                                       int height,
                                                       bool is_nv12) {
  NV_ENC_BUFFER_FORMAT nvenc_format =
      is_nv12 ? NV_ENC_BUFFER_FORMAT_NV12 : NV_ENC_BUFFER_FORMAT_IYUV;
  CUcontext context = GetCudaContext(cuda_context_);
  return new NvEncoderCuda(context, width, height, nvenc_format);
}

}  // namespace sora
