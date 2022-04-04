#ifndef CUDA_CONTEXT_H_
#define CUDA_CONTEXT_H_

#include <memory>

// <cuda.h> に依存せずに CUDA のコンテキストを表す
class CudaContext {
 public:
  static std::shared_ptr<CudaContext> Create();
  void* Context() const;

 private:
  std::shared_ptr<void> impl_;
};

enum class CudaVideoCodec {
  H264,
  VP8,
  VP9,
  JPEG,
};

#endif