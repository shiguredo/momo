#ifndef SORA_CUDA_CONTEXT_CUDA_H_
#define SORA_CUDA_CONTEXT_CUDA_H_

#include <cuda.h>

#include "sora/cuda_context.h"

namespace sora {

CUcontext GetCudaContext(std::shared_ptr<CudaContext> ctx);

}

#endif