#include <memory>

#include "sora/cuda_context.h"

#if !defined(USE_NVCODEC_ENCODER)

namespace sora {

std::shared_ptr<CudaContext> CudaContext::Create() {
  return nullptr;
}
bool CudaContext::CanCreate() {
  return false;
}

}  // namespace sora

#else

#include "sora/fix_cuda_noinline_macro_error.h"

#include <exception>

// CUDA
#include <cuda.h>

// NvCodec
#include "NvEncoder/../../Utils/Logger.h"
#include "NvEncoder/../../Utils/NvCodecUtils.h"

#include "cuda_context_cuda.h"
#include "sora/dyn/cuda.h"
#include "sora/dyn/dyn.h"

// どこかにグローバルな logger の定義が必要
simplelogger::Logger* logger =
    simplelogger::LoggerFactory::CreateConsoleLogger();

namespace sora {

struct CudaContextImpl : CudaContext {
  CUdevice device;
  CUcontext context;
  ~CudaContextImpl() { dyn::cuCtxDestroy(context); }
};

#define ckerror(call) \
  if (!ck(call))      \
  throw std::exception()

std::shared_ptr<CudaContext> CudaContext::Create() {
  try {
    CUdevice device;
    CUcontext context;

    if (!dyn::DynModule::Instance().IsLoadable(dyn::CUDA_SO)) {
      throw std::exception();
    }

    ckerror(dyn::cuInit(0));
    ckerror(dyn::cuDeviceGet(&device, 0));
    char device_name[80];
    ckerror(dyn::cuDeviceGetName(device_name, sizeof(device_name), device));
    //RTC_LOG(LS_INFO) << "GPU in use: " << device_name;
    // CUDA 13 のヘッダでは cuCtxCreate が 4 引数。momo は CUDA 12 を使う
#if CUDA_VERSION >= 13000
    ckerror(dyn::cuCtxCreate(&context, nullptr, 0, device));
#else
    ckerror(dyn::cuCtxCreate(&context, 0, device));
#endif

    auto impl = std::make_shared<CudaContextImpl>();
    impl->device = device;
    impl->context = context;
    return impl;
  } catch (std::exception&) {
    return nullptr;
  }
}

// Create() と同じことをするけど、エラーログを出さないようにする
bool CudaContext::CanCreate() {
  CUdevice device;
  CUcontext context;

  if (!dyn::DynModule::Instance().IsLoadable(dyn::CUDA_SO)) {
    return false;
  }

  CUresult r;
  r = dyn::cuInit(0);
  if (r != CUDA_SUCCESS) {
    return false;
  }

  r = dyn::cuDeviceGet(&device, 0);
  if (r != CUDA_SUCCESS) {
    return false;
  }

#if CUDA_VERSION >= 13000
  r = dyn::cuCtxCreate(&context, nullptr, 0, device);
#else
  r = dyn::cuCtxCreate(&context, 0, device);
#endif
  if (r != CUDA_SUCCESS) {
    return false;
  }

  dyn::cuCtxDestroy(context);

  return true;
}

CUdevice GetCudaDevice(std::shared_ptr<CudaContext> ctx) {
  return std::static_pointer_cast<CudaContextImpl>(ctx)->device;
}

CUcontext GetCudaContext(std::shared_ptr<CudaContext> ctx) {
  return std::static_pointer_cast<CudaContextImpl>(ctx)->context;
}

}  // namespace sora

#endif
