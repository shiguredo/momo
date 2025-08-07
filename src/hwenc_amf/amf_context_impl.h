#ifndef MOMO_AMF_CONTEXT_IMPL_H_
#define MOMO_AMF_CONTEXT_IMPL_H_

#include <memory>

namespace amf {
class AMFFactory;
}

namespace momo {

class AMFContext;

class AMFFactoryHelper {
 public:
  virtual ~AMFFactoryHelper() = default;
  virtual amf::AMFFactory* GetFactory() = 0;
};

AMFFactoryHelper* GetAMFFactoryHelper(std::shared_ptr<AMFContext> ctx);

}  // namespace momo

#endif  // MOMO_AMF_CONTEXT_IMPL_H_