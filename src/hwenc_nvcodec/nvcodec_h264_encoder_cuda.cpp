#include "nvcodec_h264_encoder_cuda.h"

#include <iostream>

// NvCodec
#include <NvDecoder/NvDecoder.h>
#include <NvEncoder/NvEncoderCuda.h>

#include "dyn/cuda.h"

// どこかにグローバルな logger の定義が必要
simplelogger::Logger* logger =
    simplelogger::LoggerFactory::CreateConsoleLogger();

class NvCodecH264EncoderCudaImpl {
 public:
  NvCodecH264EncoderCudaImpl();
  ~NvCodecH264EncoderCudaImpl();

  void Copy(NvEncoder* nv_encoder, const void* ptr, int width, int height);
  void CopyNative(NvEncoder* nv_encoder,
                  const void* ptr,
                  int size,
                  int width,
                  int height);
  NvEncoder* CreateNvEncoder(int width, int height, bool use_native);

 private:
  NvDecoder* nv_decoder_ = nullptr;
  CUdevice cu_device_;
  CUcontext cu_context_;
};

NvCodecH264EncoderCuda::NvCodecH264EncoderCuda()
    : impl_(new NvCodecH264EncoderCudaImpl()) {}
NvCodecH264EncoderCuda::~NvCodecH264EncoderCuda() {
  delete impl_;
}

void NvCodecH264EncoderCuda::Copy(NvEncoder* nv_encoder,
                                  const void* ptr,
                                  int width,
                                  int height) {
  impl_->Copy(nv_encoder, ptr, width, height);
}
void NvCodecH264EncoderCuda::CopyNative(NvEncoder* nv_encoder,
                                        const void* ptr,
                                        int size,
                                        int width,
                                        int height) {
  impl_->CopyNative(nv_encoder, ptr, size, width, height);
}
NvEncoder* NvCodecH264EncoderCuda::CreateNvEncoder(int width,
                                                   int height,
                                                   bool use_native) {
  return impl_->CreateNvEncoder(width, height, use_native);
}

void ShowEncoderCapability() {
  ck(dyn::cuInit(0));
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

NvCodecH264EncoderCudaImpl::NvCodecH264EncoderCudaImpl() {
  ShowEncoderCapability();

  ck(dyn::cuInit(0));
  ck(dyn::cuDeviceGet(&cu_device_, 0));
  char device_name[80];
  ck(dyn::cuDeviceGetName(device_name, sizeof(device_name), cu_device_));
  std::cout << "GPU in use: " << device_name << std::endl;
  ck(dyn::cuCtxCreate(&cu_context_, 0, cu_device_));
}
NvCodecH264EncoderCudaImpl::~NvCodecH264EncoderCudaImpl() {
  if (nv_decoder_ != nullptr) {
    delete nv_decoder_;
  }
  dyn::cuCtxDestroy(cu_context_);
}
void NvCodecH264EncoderCudaImpl::Copy(NvEncoder* nv_encoder,
                                      const void* ptr,
                                      int width,
                                      int height) {
  const NvEncInputFrame* input_frame = nv_encoder->GetNextInputFrame();
  NvEncoderCuda::CopyToDeviceFrame(
      cu_context_, (void*)ptr, 0, (CUdeviceptr)input_frame->inputPtr,
      (int)input_frame->pitch, width, height, CU_MEMORYTYPE_HOST,
      input_frame->bufferFormat, input_frame->chromaOffsets,
      input_frame->numChromaPlanes);
}
void NvCodecH264EncoderCudaImpl::CopyNative(NvEncoder* nv_encoder,
                                            const void* ptr,
                                            int size,
                                            int width,
                                            int height) {
  if (nv_decoder_ == nullptr) {
    std::cout << "Use JPEG Decoder" << std::endl;
    nv_decoder_ = new NvDecoder(cu_context_, true, cudaVideoCodec_JPEG, false,
                                true, nullptr, nullptr, 3840, 2160);
  }
  int frame_count = nv_decoder_->Decode((const uint8_t*)ptr, size);

  for (int i = 0; i < frame_count; i++) {
    uint8_t* frame = nv_decoder_->GetFrame();
    const NvEncInputFrame* input_frame = nv_encoder->GetNextInputFrame();
    NvEncoderCuda::CopyToDeviceFrame(
        cu_context_, frame, nv_decoder_->GetDeviceFramePitch(),
        (CUdeviceptr)input_frame->inputPtr, (int)input_frame->pitch, width,
        height, CU_MEMORYTYPE_DEVICE, input_frame->bufferFormat,
        input_frame->chromaOffsets, input_frame->numChromaPlanes);
  }
}
NvEncoder* NvCodecH264EncoderCudaImpl::CreateNvEncoder(int width,
                                                       int height,
                                                       bool use_native) {
  NV_ENC_BUFFER_FORMAT nvenc_format =
      use_native ? NV_ENC_BUFFER_FORMAT_NV12 : NV_ENC_BUFFER_FORMAT_IYUV;
  return new NvEncoderCuda(cu_context_, width, height, nvenc_format);
}
