#ifndef MSDK_SESSION_H_
#define MSDK_SESSION_H_

#include <memory>

// Intel Media SDK
#include <mfx/mfxvideo++.h>

#ifdef __linux__
#include "vaapi_utils_drm.h"
#endif

#ifdef _WIN32
#include <d3d11.h>
#include <wrl.h>
#endif

struct MsdkSession {
  MFXVideoSession session;

  ~MsdkSession();

#ifdef __linux__
  std::unique_ptr<DRMLibVA> libva;
#endif
#ifdef _WIN32
  Microsoft::WRL::ComPtr<ID3D11Device> d3d11_device;
  Microsoft::WRL::ComPtr<ID3D11DeviceContext> d3d11_context;
#endif

  static std::shared_ptr<MsdkSession> Create();
};

#endif