#ifndef AMF_CONTEXT_H_
#define AMF_CONTEXT_H_

#include <memory>

namespace sora {

class AMFContext {
 public:
  // AMF のコンテキストを生成する。
  // AMF に対応していないプラットフォームでは nullptr を返す。
  static std::shared_ptr<AMFContext> Create();
  static bool CanCreate();
};

}  // namespace sora

#endif
