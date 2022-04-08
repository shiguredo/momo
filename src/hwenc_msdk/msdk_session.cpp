#include "msdk_session.h"

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
    return nullptr;
  }

  Microsoft::WRL::ComPtr<IDXGIFactory1> idxgi_factory;
  if (FAILED(CreateDXGIFactory1(__uuidof(IDXGIFactory1),
                                (void**)idxgi_factory.GetAddressOf()))) {
    return nullptr;
  }
  Microsoft::WRL::ComPtr<IDXGIAdapter> idxgi_adapter;
  if (FAILED(idxgi_factory->EnumAdapters(0, idxgi_adapter.GetAddressOf()))) {
    return nullptr;
  }
  if (FAILED(D3D11CreateDevice(idxgi_adapter.Get(), D3D_DRIVER_TYPE_UNKNOWN,
                               NULL, 0, NULL, 0, D3D11_SDK_VERSION,
                               session->d3d11_device.GetAddressOf(), NULL,
                               session->d3d11_context.GetAddressOf()))) {
    return nullptr;
  }

  sts = session->session.SetHandle(MFX_HANDLE_D3D11_DEVICE,
                                   session->d3d11_device.Get());
  if (sts != MFX_ERR_NONE) {
    return nullptr;
  }
#endif

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