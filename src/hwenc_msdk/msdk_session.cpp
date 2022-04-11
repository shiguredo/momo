#include "msdk_session.h"

#include <iostream>

MsdkSession::~MsdkSession() {
  session.Close();
}

#ifdef _WIN32

#include <dxgi1_2.h>

const struct {
  mfxIMPL impl;      // actual implementation
  mfxU32 adapterID;  // device adapter number
} implTypes[] = {{MFX_IMPL_HARDWARE, 0},
                 {MFX_IMPL_HARDWARE2, 1},
                 {MFX_IMPL_HARDWARE3, 2},
                 {MFX_IMPL_HARDWARE4, 3}};

static IDXGIAdapter* GetIntelDeviceAdapterHandle(mfxSession session) {
  mfxU32 adapterNum = 0;
  mfxIMPL impl;

  MFXQueryIMPL(session, &impl);

  mfxIMPL baseImpl =
      MFX_IMPL_BASETYPE(impl);  // Extract Media SDK base implementation type

  // get corresponding adapter number
  for (mfxU8 i = 0; i < sizeof(implTypes) / sizeof(implTypes[0]); i++) {
    if (implTypes[i].impl == baseImpl) {
      adapterNum = implTypes[i].adapterID;
      break;
    }
  }

  Microsoft::WRL::ComPtr<IDXGIFactory2> factory;
  HRESULT hres = CreateDXGIFactory(__uuidof(IDXGIFactory2),
                                   (void**)factory.GetAddressOf());
  if (FAILED(hres))
    return NULL;

  IDXGIAdapter* adapter;
  hres = factory->EnumAdapters(adapterNum, &adapter);
  if (FAILED(hres))
    return NULL;

  return adapter;
}

#endif

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
    std::cerr << "Failed to MFXInit: sts=" << sts << std::endl;
    return nullptr;
  }

  /*
  static D3D_FEATURE_LEVEL FeatureLevels[] = {
      D3D_FEATURE_LEVEL_11_1, D3D_FEATURE_LEVEL_11_0, D3D_FEATURE_LEVEL_10_1,
      D3D_FEATURE_LEVEL_10_0};
  D3D_FEATURE_LEVEL pFeatureLevelsOut;

  Microsoft::WRL::ComPtr<IDXGIAdapter> idxgi_adapter =
      GetIntelDeviceAdapterHandle(session->session);
  if (FAILED(D3D11CreateDevice(
          idxgi_adapter.Get(), D3D_DRIVER_TYPE_UNKNOWN, NULL, 0, FeatureLevels,
          (sizeof(FeatureLevels) / sizeof(FeatureLevels[0])), D3D11_SDK_VERSION,
          session->d3d11_device.GetAddressOf(), &pFeatureLevelsOut,
          session->d3d11_context.GetAddressOf()))) {
    std::cerr << "Failed to D3D11CreateDevice" << std::endl;
    return nullptr;
  }

  Microsoft::WRL::ComPtr<ID3D10Multithread> mt;
  if (FAILED(session->d3d11_context->QueryInterface(mt.GetAddressOf()))) {
    std::cerr << "Failed to QueryInterface" << std::endl;
  }
  mt->SetMultithreadProtected(true);

  sts = session->session.SetHandle(MFX_HANDLE_D3D11_DEVICE,
                                   session->d3d11_device.Get());
  if (sts != MFX_ERR_NONE) {
    std::cerr << "Failed to MFXSetHandle: sts=" << sts << std::endl;
    return nullptr;
  }
  */
#endif

  // Query selected implementation and version
  sts = session->session.QueryIMPL(&impl);
  if (sts != MFX_ERR_NONE) {
    std::cerr << "Failed to MFXQueryIMPL: sts=" << sts << std::endl;
    return nullptr;
  }

  sts = session->session.QueryVersion(&ver);
  if (sts != MFX_ERR_NONE) {
    std::cerr << "Failed to MFXQueryVersion: sts=" << sts << std::endl;
    return nullptr;
  }

  //RTC_LOG(LS_INFO) << "Intel Media SDK Implementation: "
  //                 << (impl == MFX_IMPL_SOFTWARE ? "SOFTWARE" : "HARDWARE");
  //RTC_LOG(LS_INFO) << "Intel Media SDK API Version: " << ver.Major << "."
  //                 << ver.Minor;
  return session;
}