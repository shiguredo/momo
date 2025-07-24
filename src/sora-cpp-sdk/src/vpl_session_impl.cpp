#include <memory>

#include "sora/vpl_session.h"

#if !defined(USE_VPL_ENCODER)

namespace sora {

std::shared_ptr<VplSession> VplSession::Create() {
  return nullptr;
}

}  // namespace sora

#else

#include "vpl_session_impl.h"

// WebRTC
#include <rtc_base/logging.h>

// VPL
#include <vpl/mfxcommon.h>
#include <vpl/mfxdefs.h>
#include <vpl/mfxdispatcher.h>
#include <vpl/mfxsession.h>

namespace sora {

struct VplSessionImpl : VplSession {
  ~VplSessionImpl();

  mfxLoader loader = nullptr;
  mfxSession session = nullptr;
};

VplSessionImpl::~VplSessionImpl() {
  MFXClose(session);
  MFXUnload(loader);
}

std::shared_ptr<VplSession> VplSession::Create() {
  std::shared_ptr<VplSessionImpl> session(new VplSessionImpl());

  mfxStatus sts = MFX_ERR_NONE;

  session->loader = MFXLoad();
  if (session->loader == nullptr) {
    RTC_LOG(LS_VERBOSE) << "Failed to MFXLoad";
    return nullptr;
  }

  MFX_ADD_PROPERTY_U32(session->loader, "mfxImplDescription.Impl",
                       MFX_IMPL_TYPE_HARDWARE);

  sts = MFXCreateSession(session->loader, 0, &session->session);
  if (sts != MFX_ERR_NONE) {
    RTC_LOG(LS_VERBOSE) << "Failed to MFXCreateSession: sts=" << sts;
    return nullptr;
  }

  // Query selected implementation and version
  mfxIMPL impl;
  sts = MFXQueryIMPL(session->session, &impl);
  if (sts != MFX_ERR_NONE) {
    RTC_LOG(LS_VERBOSE) << "Failed to MFXQueryIMPL: sts=" << sts;
    return nullptr;
  }

  mfxVersion ver;
  sts = MFXQueryVersion(session->session, &ver);
  if (sts != MFX_ERR_NONE) {
    RTC_LOG(LS_VERBOSE) << "Failed to MFXQueryVersion: sts=" << sts;
    return nullptr;
  }

  RTC_LOG(LS_VERBOSE) << "Intel VPL Implementation: "
                      << (impl == MFX_IMPL_SOFTWARE ? "SOFTWARE" : "HARDWARE");
  RTC_LOG(LS_VERBOSE) << "Intel VPL Version: " << ver.Major << "." << ver.Minor;
  return session;
}

mfxSession GetVplSession(std::shared_ptr<VplSession> session) {
  return std::static_pointer_cast<VplSessionImpl>(session)->session;
}

}  // namespace sora

#endif
