#include "msdk_session.h"

#include <iostream>

MsdkSession::~MsdkSession() {
  session.Close();
}

std::shared_ptr<MsdkSession> MsdkSession::Create() {
  std::shared_ptr<MsdkSession> session(new MsdkSession());

  mfxStatus sts = MFX_ERR_NONE;

  mfxVersion ver = {{0, 1}};

#ifdef __linux__
  mfxIMPL impl = MFX_IMPL_HARDWARE_ANY;
  sts = session->session.Init(impl, &ver);
  if (sts != MFX_ERR_NONE) {
    return nullptr;
  }

  session->libva = CreateDRMLibVA();
  if (!session->libva) {
    return nullptr;
  }

  sts = session->session.SetHandle(
      static_cast<mfxHandleType>(MFX_HANDLE_VA_DISPLAY),
      session->libva->GetVADisplay());
  if (sts != MFX_ERR_NONE) {
    return nullptr;
  }
#endif

#ifdef _WIN32
  mfxIMPL impl = MFX_IMPL_VIA_D3D11 | MFX_IMPL_HARDWARE_ANY;

  sts = session->session.Init(impl, &ver);
  if (sts != MFX_ERR_NONE) {
    //std::cerr << "Failed to MFXInit: sts=" << sts << std::endl;
    return nullptr;
  }

#endif

  // Query selected implementation and version
  sts = session->session.QueryIMPL(&impl);
  if (sts != MFX_ERR_NONE) {
    //std::cerr << "Failed to MFXQueryIMPL: sts=" << sts << std::endl;
    return nullptr;
  }

  sts = session->session.QueryVersion(&ver);
  if (sts != MFX_ERR_NONE) {
    //std::cerr << "Failed to MFXQueryVersion: sts=" << sts << std::endl;
    return nullptr;
  }

  //RTC_LOG(LS_INFO) << "Intel Media SDK Implementation: "
  //                 << (impl == MFX_IMPL_SOFTWARE ? "SOFTWARE" : "HARDWARE");
  //RTC_LOG(LS_INFO) << "Intel Media SDK API Version: " << ver.Major << "."
  //                 << ver.Minor;
  return session;
}