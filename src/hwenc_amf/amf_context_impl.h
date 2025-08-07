#ifndef MOMO_AMF_CONTEXT_IMPL_H_
#define MOMO_AMF_CONTEXT_IMPL_H_

#include <memory>

// AMF
#include <public/common/AMFFactory.h>

#include "amf_context.h"

namespace momo {

AMFFactoryHelper* GetAMFFactoryHelper(std::shared_ptr<AMFContext> ctx);

}  // namespace momo

#endif  // MOMO_AMF_CONTEXT_IMPL_H_