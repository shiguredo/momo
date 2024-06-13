#include "sora/fix_cuda_noinline_macro_error.h"

#include "sora/cuda_context.h"

#if !defined(USE_NVCODEC_ENCODER)

namespace sora {

std::shared_ptr<CudaContext> CudaContext::Create() {
  return nullptr;
}

void* CudaContext::Context() const {
  return nullptr;
}

}  // namespace sora

#else

#include "cuda_context_cuda.h"

// NvCodec
#include <NvDecoder/NvDecoder.h>

#include "sora/dyn/cuda.h"

// どこかにグローバルな logger の定義が必要
simplelogger::Logger* logger =
    simplelogger::LoggerFactory::CreateConsoleLogger();

namespace sora {

CUcontext GetCudaContext(std::shared_ptr<CudaContext> ctx) {
  return static_cast<CUcontext>(ctx->Context());
}

struct CudaContextImpl {
  CUdevice device;
  CUcontext context;
  ~CudaContextImpl() { dyn::cuCtxDestroy(context); }
};

#define ckerror(call) \
  if (!ck(call))      \
  throw std::exception()

std::shared_ptr<CudaContext> CudaContext::Create() {
  std::shared_ptr<CudaContext> ctx;
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
    ckerror(dyn::cuCtxCreate(&context, 0, device));

    std::shared_ptr<CudaContextImpl> impl(new CudaContextImpl());
    impl->device = device;
    impl->context = context;

    ctx.reset(new CudaContext());
    ctx->impl_ = impl;
  } catch (std::exception&) {
    return nullptr;
  }

  return ctx;
}

void* CudaContext::Context() const {
  return std::static_pointer_cast<CudaContextImpl>(impl_)->context;
}

}  // namespace sora

#endif
