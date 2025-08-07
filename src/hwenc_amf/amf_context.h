#ifndef MOMO_AMF_CONTEXT_H_
#define MOMO_AMF_CONTEXT_H_

#include <memory>

namespace momo {

class AMFContext {
 public:
  // AMF のコンテキストを生成する。
  // AMF に対応していないプラットフォームでは nullptr を返す。
  static std::shared_ptr<AMFContext> Create();
  static bool CanCreate();
};

}  // namespace momo

#endif  // MOMO_AMF_CONTEXT_H_