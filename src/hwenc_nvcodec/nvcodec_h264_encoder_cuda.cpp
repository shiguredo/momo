#include "nvcodec_h264_encoder_cuda.h"

#include <iostream>

#ifdef __linux__
#include <NvEncoder/NvEncoderCuda.h>
#endif

#ifdef __cuda_cuda_h__
inline bool check(CUresult e, int iLine, const char* szFile) {
  if (e != CUDA_SUCCESS) {
    const char* szErrName = NULL;
    cuGetErrorName(e, &szErrName);
    std::cerr << "CUDA driver API error " << szErrName << " at line " << iLine
              << " in file " << szFile << std::endl;
    return false;
  }
  return true;
}
#endif

#ifdef __CUDA_RUNTIME_H__
inline bool check(cudaError_t e, int iLine, const char* szFile) {
  if (e != cudaSuccess) {
    std::cerr << "CUDA runtime API error " << cudaGetErrorName(e) << " at line "
              << iLine << " in file " << szFile << std::endl;
    return false;
  }
  return true;
}
#endif

#ifdef _NV_ENCODEAPI_H_
inline bool check(NVENCSTATUS e, int iLine, const char* szFile) {
  const char* aszErrName[] = {
      "NV_ENC_SUCCESS",
      "NV_ENC_ERR_NO_ENCODE_DEVICE",
      "NV_ENC_ERR_UNSUPPORTED_DEVICE",
      "NV_ENC_ERR_INVALID_ENCODERDEVICE",
      "NV_ENC_ERR_INVALID_DEVICE",
      "NV_ENC_ERR_DEVICE_NOT_EXIST",
      "NV_ENC_ERR_INVALID_PTR",
      "NV_ENC_ERR_INVALID_EVENT",
      "NV_ENC_ERR_INVALID_PARAM",
      "NV_ENC_ERR_INVALID_CALL",
      "NV_ENC_ERR_OUT_OF_MEMORY",
      "NV_ENC_ERR_ENCODER_NOT_INITIALIZED",
      "NV_ENC_ERR_UNSUPPORTED_PARAM",
      "NV_ENC_ERR_LOCK_BUSY",
      "NV_ENC_ERR_NOT_ENOUGH_BUFFER",
      "NV_ENC_ERR_INVALID_VERSION",
      "NV_ENC_ERR_MAP_FAILED",
      "NV_ENC_ERR_NEED_MORE_INPUT",
      "NV_ENC_ERR_ENCODER_BUSY",
      "NV_ENC_ERR_EVENT_NOT_REGISTERD",
      "NV_ENC_ERR_GENERIC",
      "NV_ENC_ERR_INCOMPATIBLE_CLIENT_KEY",
      "NV_ENC_ERR_UNIMPLEMENTED",
      "NV_ENC_ERR_RESOURCE_REGISTER_FAILED",
      "NV_ENC_ERR_RESOURCE_NOT_REGISTERED",
      "NV_ENC_ERR_RESOURCE_NOT_MAPPED",
  };
  if (e != NV_ENC_SUCCESS) {
    std::cerr << "NVENC error " << aszErrName[e] << " at line " << iLine
              << " in file " << szFile << std::endl;
    return false;
  }
  return true;
}
#endif

#ifdef _WINERROR_
inline bool check(HRESULT e, int iLine, const char* szFile) {
  if (e != S_OK) {
    std::cerr << "HRESULT error 0x" << (void*)e << " at line " << iLine
              << " in file " << szFile << std::endl;
    return false;
  }
  return true;
}
#endif

#if defined(__gl_h_) || defined(__GL_H__)
inline bool check(GLenum e, int iLine, const char* szFile) {
  if (e != 0) {
    std::cerr << "GLenum error " << e << " at line " << iLine << " in file "
              << szFile << std::endl;
    return false;
  }
  return true;
}
#endif

inline bool check(int e, int iLine, const char* szFile) {
  if (e < 0) {
    std::cerr << "General error " << e << " at line " << iLine << " in file "
              << szFile << std::endl;
    return false;
  }
  return true;
}

#define ck(call) check(call, __LINE__, __FILE__)

class NvCodecH264EncoderCudaImpl {
 public:
  NvCodecH264EncoderCudaImpl();
  ~NvCodecH264EncoderCudaImpl();

  void Copy(const NvEncInputFrame* input_frame,
            const void* ptr,
            int width,
            int height);
  NvEncoder* CreateNvEncoder(int width, int height);

 private:
  CUdevice cu_device_;
  CUcontext cu_context_;
};

NvCodecH264EncoderCuda::NvCodecH264EncoderCuda()
    : impl_(new NvCodecH264EncoderCudaImpl()) {}
NvCodecH264EncoderCuda::~NvCodecH264EncoderCuda() {
  delete impl_;
}

void NvCodecH264EncoderCuda::Copy(const NvEncInputFrame* input_frame,
                                  const void* ptr,
                                  int width,
                                  int height) {
  impl_->Copy(input_frame, ptr, width, height);
}
NvEncoder* NvCodecH264EncoderCuda::CreateNvEncoder(int width, int height) {
  return impl_->CreateNvEncoder(width, height);
}

void ShowEncoderCapability() {
  ck(cuInit(0));
  int nGpu = 0;
  ck(cuDeviceGetCount(&nGpu));
  if (nGpu == 0) {
    std::cerr << "CUDA Device not found" << std::endl;
    exit(1);
  }
  std::cout << "Encoder Capability" << std::endl;
  for (int iGpu = 0; iGpu < nGpu; iGpu++) {
    CUdevice cuDevice = 0;
    ck(cuDeviceGet(&cuDevice, iGpu));
    char szDeviceName[80];
    ck(cuDeviceGetName(szDeviceName, sizeof(szDeviceName), cuDevice));
    CUcontext cuContext = NULL;
    ck(cuCtxCreate(&cuContext, 0, cuDevice));
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
    ck(cuCtxDestroy(cuContext));
  }
}

NvCodecH264EncoderCudaImpl::NvCodecH264EncoderCudaImpl() {
  ShowEncoderCapability();

  ck(cuInit(0));
  ck(cuDeviceGet(&cu_device_, 0));
  char device_name[80];
  ck(cuDeviceGetName(device_name, sizeof(device_name), cu_device_));
  std::cout << "GPU in use: " << device_name;
  ck(cuCtxCreate(&cu_context_, 0, cu_device_));
}
NvCodecH264EncoderCudaImpl::~NvCodecH264EncoderCudaImpl() {
  cuCtxDestroy(cu_context_);
}
void NvCodecH264EncoderCudaImpl::Copy(const NvEncInputFrame* input_frame,
                                      const void* ptr,
                                      int width,
                                      int height) {
  NvEncoderCuda::CopyToDeviceFrame(
      cu_context_, (void*)ptr, 0, (CUdeviceptr)input_frame->inputPtr,
      (int)input_frame->pitch, width, height, CU_MEMORYTYPE_HOST,
      input_frame->bufferFormat, input_frame->chromaOffsets,
      input_frame->numChromaPlanes);
}
NvEncoder* NvCodecH264EncoderCudaImpl::CreateNvEncoder(int width, int height) {
  NV_ENC_BUFFER_FORMAT nvenc_format = NV_ENC_BUFFER_FORMAT_IYUV;
  return new NvEncoderCuda(cu_context_, width, height, nvenc_format);
}
