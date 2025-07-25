#ifndef SORA_VPL_SESSION_IMPL_H_
#define SORA_VPL_SESSION_IMPL_H_

#if defined(USE_VPL_ENCODER)

#include <memory>

// Intel VPL
#include <vpl/mfxsession.h>

#include "sora/vpl_session.h"

namespace sora {

mfxSession GetVplSession(std::shared_ptr<VplSession> session);

}  // namespace sora

#endif

#endif