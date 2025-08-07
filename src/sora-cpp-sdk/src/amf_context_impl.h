#ifndef SORA_AMF_CONTEXT_IMPL_H_
#define SORA_AMF_CONTEXT_IMPL_H_

#include <memory>

// AMF
#include <public/common/AMFFactory.h>

#include "sora/amf_context.h"

namespace sora {

AMFFactoryHelper* GetAMFFactoryHelper(std::shared_ptr<AMFContext> ctx);

}  // namespace sora

#endif