#ifndef VPL_SESSION_H_
#define VPL_SESSION_H_

#include <memory>

// oneVPL
#include <vpl/mfxvideo++.h>

#ifdef __linux__
#include "vaapi_utils_drm.h"
#endif

struct VplSession {
  MFXVideoSession session;

  ~VplSession();

#ifdef __linux__
  std::unique_ptr<DRMLibVA> libva;
#endif

  static std::shared_ptr<VplSession> Create();
};

#endif
