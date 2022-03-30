#ifndef CUDA_CONTEXT_CUDA_H_
#define CUDA_CONTEXT_CUDA_H_

#include <cuda.h>

#include "cuda_context.h"

CUcontext GetCudaContext(std::shared_ptr<CudaContext> ctx);

#endif