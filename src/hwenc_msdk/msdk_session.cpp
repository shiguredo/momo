#include "msdk_session.h"

std::shared_ptr<MsdkSession> MsdkSession::Create() {
  std::shared_ptr<MsdkSession> session(new MsdkSession());
  session->libva = CreateDRMLibVA();
  if (!session->libva) {
    return nullptr;
  }

  mfxStatus sts = MFX_ERR_NONE;

  mfxIMPL impl = MFX_IMPL_HARDWARE_ANY;
  mfxVersion ver = {{0, 1}};

  sts = session->session.Init(impl, &ver);
  if (sts != MFX_ERR_NONE) {
    return nullptr;
  }

  sts = session->session.SetHandle(
      static_cast<mfxHandleType>(MFX_HANDLE_VA_DISPLAY),
      session->libva->GetVADisplay());
  if (sts != MFX_ERR_NONE) {
    return nullptr;
  }

  // Query selected implementation and version
  sts = session->session.QueryIMPL(&impl);
  if (sts != MFX_ERR_NONE) {
    return nullptr;
  }

  sts = session->session.QueryVersion(&ver);
  if (sts != MFX_ERR_NONE) {
    return nullptr;
  }

  //RTC_LOG(LS_INFO) << "Intel Media SDK Implementation: "
  //                 << (impl == MFX_IMPL_SOFTWARE ? "SOFTWARE" : "HARDWARE");
  //RTC_LOG(LS_INFO) << "Intel Media SDK API Version: " << ver.Major << "."
  //                 << ver.Minor;
  return session;
}