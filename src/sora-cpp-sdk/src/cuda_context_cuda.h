#ifndef SORA_CUDA_CONTEXT_CUDA_H_
#define SORA_CUDA_CONTEXT_CUDA_H_

#include <memory>

// CUDA
#include <cuda.h>

#include "sora/cuda_context.h"

namespace sora {

CUdevice GetCudaDevice(std::shared_ptr<CudaContext> ctx);
CUcontext GetCudaContext(std::shared_ptr<CudaContext> ctx);

}  // namespace sora

#endif