#include "sora/hwenc_vpl/vpl_frame_allocator.h"

#include <cstring>

// WebRTC
#include <rtc_base/logging.h>

// VA-API
#include <va/va_drmcommon.h>

namespace sora {

VplFrameAllocator::VplFrameAllocator() : va_display_(nullptr) {
  // 基本クラスのコールバック関数ポインタを設定
  Alloc = [](mfxHDL pthis, mfxFrameAllocRequest* request,
             mfxFrameAllocResponse* response) -> mfxStatus {
    return static_cast<VplFrameAllocator*>(pthis)->Alloc(request, response);
  };

  Lock = [](mfxHDL pthis, mfxMemId mid, mfxFrameData* ptr) -> mfxStatus {
    return static_cast<VplFrameAllocator*>(pthis)->Lock(mid, ptr);
  };

  Unlock = [](mfxHDL pthis, mfxMemId mid, mfxFrameData* ptr) -> mfxStatus {
    return static_cast<VplFrameAllocator*>(pthis)->Unlock(mid, ptr);
  };

  GetHDL = [](mfxHDL pthis, mfxMemId mid, mfxHDL* handle) -> mfxStatus {
    return static_cast<VplFrameAllocator*>(pthis)->GetHDL(mid, handle);
  };

  Free = [](mfxHDL pthis, mfxFrameAllocResponse* response) -> mfxStatus {
    return static_cast<VplFrameAllocator*>(pthis)->Free(response);
  };

  pthis = this;
}

VplFrameAllocator::~VplFrameAllocator() {
  if (alloc_response_.NumFrameActual > 0) {
    Free(&alloc_response_);
  }
}

mfxStatus VplFrameAllocator::Init(VADisplay va_display) {
  va_display_ = va_display;
  return MFX_ERR_NONE;
}

mfxStatus VplFrameAllocator::Alloc(mfxFrameAllocRequest* request,
                                   mfxFrameAllocResponse* response) {
  if (!va_display_) {
    RTC_LOG(LS_ERROR) << "VA display not initialized";
    return MFX_ERR_NOT_INITIALIZED;
  }

  // YUY2 フォーマットをサポート
  VASurfaceAttrib attrib = {};
  attrib.type = VASurfaceAttribPixelFormat;
  attrib.flags = VA_SURFACE_ATTRIB_SETTABLE;
  attrib.value.type = VAGenericValueTypeInteger;

  // フォーマット判定
  unsigned int rt_format;
  if (request->Info.FourCC == MFX_FOURCC_YUY2) {
    attrib.value.value.i = VA_FOURCC_YUY2;
    rt_format = VA_RT_FORMAT_YUV422;
  } else if (request->Info.FourCC == MFX_FOURCC_NV12) {
    attrib.value.value.i = VA_FOURCC_NV12;
    rt_format = VA_RT_FORMAT_YUV420;
  } else {
    RTC_LOG(LS_ERROR) << "Unsupported FourCC: " << request->Info.FourCC;
    return MFX_ERR_UNSUPPORTED;
  }

  // VA サーフェースを作成
  std::vector<VASurfaceID> va_surfaces(request->NumFrameSuggested);
  VAStatus va_status = vaCreateSurfaces(
      va_display_, rt_format, request->Info.Width, request->Info.Height,
      va_surfaces.data(), request->NumFrameSuggested, &attrib, 1);

  if (va_status != VA_STATUS_SUCCESS) {
    RTC_LOG(LS_ERROR) << "Failed to create VA surfaces: " << va_status;
    return MFX_ERR_MEMORY_ALLOC;
  }

  // Surface 構造体を作成して DMABUF fd をエクスポート
  surfaces_.clear();
  surfaces_.reserve(request->NumFrameSuggested);

  for (int i = 0; i < request->NumFrameSuggested; i++) {
    auto surface = std::make_unique<Surface>();
    surface->va_surface_id = va_surfaces[i];
    surface->locked = false;

    // DMABUF fd をエクスポート
    VADRMPRIMESurfaceDescriptor desc;
    va_status = vaExportSurfaceHandle(
        va_display_, va_surfaces[i], VA_SURFACE_ATTRIB_MEM_TYPE_DRM_PRIME_2,
        VA_EXPORT_SURFACE_READ_WRITE | VA_EXPORT_SURFACE_SEPARATE_LAYERS,
        &desc);

    if (va_status != VA_STATUS_SUCCESS) {
      RTC_LOG(LS_ERROR) << "Failed to export DMABUF: " << va_status;
      // クリーンアップ
      for (auto& s : surfaces_) {
        vaDestroySurfaces(va_display_, &s->va_surface_id, 1);
      }
      vaDestroySurfaces(va_display_, &va_surfaces[i],
                        request->NumFrameSuggested - i);
      return MFX_ERR_MEMORY_ALLOC;
    }

    // 最初のレイヤーの fd を保存
    surface->dmabuf_fd = desc.objects[0].fd;
    surfaces_.push_back(std::move(surface));

    // 他のオブジェクトの fd をクローズ（必要に応じて）
    for (uint32_t j = 1; j < desc.num_objects; j++) {
      close(desc.objects[j].fd);
    }
  }

  // レスポンスを設定
  response->NumFrameActual = request->NumFrameSuggested;
  response->mids = reinterpret_cast<mfxMemId*>(
      malloc(sizeof(mfxMemId) * request->NumFrameSuggested));

  for (int i = 0; i < request->NumFrameSuggested; i++) {
    response->mids[i] = surfaces_[i].get();
  }

  // 内部的にも保存
  alloc_response_ = *response;

  RTC_LOG(LS_INFO) << "Allocated " << request->NumFrameSuggested
                   << " VA surfaces with DMABUF support";

  return MFX_ERR_NONE;
}

mfxStatus VplFrameAllocator::Lock(mfxMemId mid, mfxFrameData* ptr) {
  auto* surface = static_cast<Surface*>(mid);
  if (!surface) {
    return MFX_ERR_NULL_PTR;
  }

  if (surface->locked) {
    return MFX_ERR_MORE_DATA;
  }

  surface->locked = true;

  // VA-API サーフェースの場合、直接マップは不要
  // VPL が内部的に処理する
  ptr->MemId = mid;

  return MFX_ERR_NONE;
}

mfxStatus VplFrameAllocator::Unlock(mfxMemId mid, mfxFrameData* ptr) {
  auto* surface = static_cast<Surface*>(mid);
  if (!surface) {
    return MFX_ERR_NULL_PTR;
  }

  surface->locked = false;
  return MFX_ERR_NONE;
}

mfxStatus VplFrameAllocator::GetHDL(mfxMemId mid, mfxHDL* handle) {
  auto* surface = static_cast<Surface*>(mid);
  if (!surface) {
    return MFX_ERR_NULL_PTR;
  }

  // VA サーフェース ID を返す
  *handle = reinterpret_cast<mfxHDL>(&surface->va_surface_id);
  return MFX_ERR_NONE;
}

mfxStatus VplFrameAllocator::Free(mfxFrameAllocResponse* response) {
  if (!response || !response->mids) {
    return MFX_ERR_NULL_PTR;
  }

  // VA サーフェースを破棄
  for (int i = 0; i < response->NumFrameActual; i++) {
    auto* surface = static_cast<Surface*>(response->mids[i]);
    if (surface) {
      vaDestroySurfaces(va_display_, &surface->va_surface_id, 1);
      close(surface->dmabuf_fd);
    }
  }

  surfaces_.clear();
  free(response->mids);
  response->mids = nullptr;
  response->NumFrameActual = 0;

  return MFX_ERR_NONE;
}

int VplFrameAllocator::GetDmaBufFd(mfxMemId mid) {
  auto* surface = static_cast<Surface*>(mid);
  if (!surface) {
    return -1;
  }
  return surface->dmabuf_fd;
}

std::vector<int> VplFrameAllocator::GetDmaBufFds() {
  std::vector<int> fds;
  fds.reserve(surfaces_.size());
  for (const auto& surface : surfaces_) {
    fds.push_back(surface->dmabuf_fd);
  }
  return fds;
}

}  // namespace sora