#ifndef DYN_CUDA_H_
#define DYN_CUDA_H_

#include <cuda.h>

#include "dyn.h"

namespace dyn {

#if defined(WIN32)
static const char CUDA_SO[] = "nvcuda.dll";
#else
static const char CUDA_SO[] = "libcuda.so.1";
#endif
DYN_REGISTER(CUDA_SO, cuInit);
DYN_REGISTER(CUDA_SO, cuDeviceGet);
DYN_REGISTER(CUDA_SO, cuDeviceGetCount);
DYN_REGISTER(CUDA_SO, cuDeviceGetName);
DYN_REGISTER(CUDA_SO, cuCtxCreate);
DYN_REGISTER(CUDA_SO, cuCtxDestroy);
DYN_REGISTER(CUDA_SO, cuCtxPushCurrent);
DYN_REGISTER(CUDA_SO, cuCtxPopCurrent);
DYN_REGISTER(CUDA_SO, cuGetErrorName);
DYN_REGISTER(CUDA_SO, cuMemAlloc);
DYN_REGISTER(CUDA_SO, cuMemAllocPitch);
DYN_REGISTER(CUDA_SO, cuMemFree);
DYN_REGISTER(CUDA_SO, cuMemcpy2D);
DYN_REGISTER(CUDA_SO, cuMemcpy2DAsync);
DYN_REGISTER(CUDA_SO, cuMemcpy2DUnaligned);
DYN_REGISTER(CUDA_SO, cuStreamSynchronize);
DYN_REGISTER(CUDA_SO, cuStreamCreate);

}  // namespace dyn

#endif  // DYN_CUDA_H_