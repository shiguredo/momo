#include "cuda_context_cuda.h"

#include <NvDecoder/NvDecoder.h>

#include "dyn/cuda.h"

CUcontext GetCudaContext(std::shared_ptr<CudaContext> ctx) {
  return static_cast<CUcontext>(ctx->Context());
}

struct CudaContextImpl {
  CUdevice device;
  CUcontext context;
  ~CudaContextImpl() { dyn::cuCtxDestroy(context); }
};

std::shared_ptr<CudaContext> CudaContext::Create() {
  CUdevice device;
  CUcontext context;

  ck(dyn::cuInit(0));
  ck(dyn::cuDeviceGet(&device, 0));
  char device_name[80];
  ck(dyn::cuDeviceGetName(device_name, sizeof(device_name), device));
  std::cout << "GPU in use: " << device_name << std::endl;
  ck(dyn::cuCtxCreate(&context, 0, device));

  std::shared_ptr<CudaContextImpl> impl(new CudaContextImpl());
  impl->device = device;
  impl->context = context;

  std::shared_ptr<CudaContext> ctx(new CudaContext());
  ctx->impl_ = impl;

  return ctx;
}

void* CudaContext::Context() const {
  return std::static_pointer_cast<CudaContextImpl>(impl_)->context;
}