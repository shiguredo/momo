#ifndef MSDK_SESSION_H_
#define MSDK_SESSION_H_

#include <memory>

// Intel Media SDK
#include <vpl/mfxvideo++.h>

#ifdef __linux__
#include "vaapi_utils_drm.h"
#endif

struct MsdkSession {
  MFXVideoSession session;

  ~MsdkSession();

#ifdef __linux__
  std::unique_ptr<DRMLibVA> libva;
#endif

  static std::shared_ptr<MsdkSession> Create();
};

#endif