#include <memory>

#include "sora/amf_context.h"

#if !defined(USE_AMF_ENCODER)

namespace sora {

std::shared_ptr<AMFContext> AMFContext::Create() {
  return nullptr;
}
bool AMFContext::CanCreate() {
  return false;
}

}  // namespace sora

#else

// AMF
#include <public/common/AMFFactory.h>
#include <public/include/core/Result.h>

#include "amf_context_impl.h"

namespace sora {

struct AMFContextImpl : AMFContext {
  ~AMFContextImpl() { g_AMFFactory.Terminate(); }
};

std::shared_ptr<AMFContext> AMFContext::Create() {
  AMF_RESULT res = g_AMFFactory.Init();
  if (res != AMF_OK) {
    return nullptr;
  }

  return std::make_shared<AMFContextImpl>();
}

bool AMFContext::CanCreate() {
  AMF_RESULT res = g_AMFFactory.Init();
  if (res != AMF_OK) {
    return false;
  }
  g_AMFFactory.Terminate();
  return true;
}

AMFFactoryHelper* GetAMFFactoryHelper(std::shared_ptr<AMFContext> ctx) {
  return &g_AMFFactory;
}

}  // namespace sora

#endif
