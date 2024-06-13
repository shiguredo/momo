#ifndef SORA_HWENC_VPL_VPL_SESSION_H_
#define SORA_HWENC_VPL_VPL_SESSION_H_

#include <memory>

namespace sora {

struct VplSession {
  static std::shared_ptr<VplSession> Create();
};

}  // namespace sora

#endif
