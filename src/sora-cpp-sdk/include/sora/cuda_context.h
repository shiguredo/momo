#ifndef CUDA_CONTEXT_H_
#define CUDA_CONTEXT_H_

#include <memory>

namespace sora {

// <cuda.h> に依存せずに CUDA のコンテキストを表す
class CudaContext {
 public:
  // CUDA コンテキスト生成する。
  // CUDA に対応していないプラットフォームでは nullptr を返す。
  static std::shared_ptr<CudaContext> Create();
  static bool CanCreate();
};

enum class CudaVideoCodec {
  H264,
  H265,
  VP8,
  VP9,
  AV1,
  JPEG,
};

}  // namespace sora

#endif