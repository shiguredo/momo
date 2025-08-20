#include "sora/vpl_surface_pool.h"
#include <rtc_base/logging.h>
#include <cstring>

namespace sora {

// YUY2 フォーマット関連の定数（vpl_video_encoder.cpp と同じ）
constexpr size_t YUY2_BYTES_PER_PIXEL = 2;
constexpr size_t YUY2_U_OFFSET = 1;
constexpr size_t YUY2_V_OFFSET = 3;

void VplSurfacePool::Initialize(int width, int height, int num_surfaces, bool use_yuy2) {
  std::lock_guard<std::mutex> lock(mutex_);
  
  if (initialized_) {
    RTC_LOG(LS_WARNING) << "VplSurfacePool already initialized, clearing...";
    Clear();
  }

  width_ = width;
  height_ = height;
  use_yuy2_ = use_yuy2;

  // アライメント（32バイト境界）
  int aligned_width = (width + 31) / 32 * 32;
  int aligned_height = (height + 31) / 32 * 32;

  // 1枚あたりのバイト数
  int size_per_surface;
  if (use_yuy2) {
    size_per_surface = aligned_width * aligned_height * YUY2_BYTES_PER_PIXEL;
  } else {
    size_per_surface = aligned_width * aligned_height * 12 / 8;  // NV12
  }

  // 全サーフェース分のメモリを確保
  surface_buffer_.resize(num_surfaces * size_per_surface);
  surfaces_.clear();
  surfaces_.reserve(num_surfaces);

  // 各サーフェースを初期化
  for (int i = 0; i < num_surfaces; i++) {
    mfxFrameSurface1 surface;
    memset(&surface, 0, sizeof(surface));
    
    // フレーム情報を設定
    surface.Info.Width = width;
    surface.Info.Height = height;
    surface.Info.CropX = 0;
    surface.Info.CropY = 0;
    surface.Info.CropW = width;
    surface.Info.CropH = height;
    
    if (use_yuy2) {
      surface.Info.FourCC = MFX_FOURCC_YUY2;
      surface.Info.ChromaFormat = MFX_CHROMAFORMAT_YUV422;
      surface.Data.Y = surface_buffer_.data() + i * size_per_surface;
      surface.Data.U = surface.Data.Y + YUY2_U_OFFSET;
      surface.Data.V = surface.Data.Y + YUY2_V_OFFSET;
      surface.Data.Pitch = aligned_width * YUY2_BYTES_PER_PIXEL;
    } else {
      surface.Info.FourCC = MFX_FOURCC_NV12;
      surface.Info.ChromaFormat = MFX_CHROMAFORMAT_YUV420;
      surface.Data.Y = surface_buffer_.data() + i * size_per_surface;
      surface.Data.U = surface.Data.Y + aligned_width * aligned_height;
      surface.Data.V = surface.Data.U + 1;
      surface.Data.Pitch = aligned_width;
    }
    
    surface.Data.Locked = 0;  // 初期状態はアンロック
    surfaces_.push_back(surface);
  }

  initialized_ = true;
  RTC_LOG(LS_INFO) << "VplSurfacePool initialized: " 
                   << width << "x" << height 
                   << " format=" << (use_yuy2 ? "YUY2" : "NV12")
                   << " surfaces=" << num_surfaces;
}

mfxFrameSurface1* VplSurfacePool::AcquireSurface() {
  std::lock_guard<std::mutex> lock(mutex_);
  
  if (!initialized_) {
    RTC_LOG(LS_ERROR) << "VplSurfacePool not initialized";
    return nullptr;
  }

  // ロックされていないサーフェスを探す
  for (auto& surface : surfaces_) {
    if (!surface.Data.Locked) {
      surface.Data.Locked = 1;
      RTC_LOG(LS_VERBOSE) << "Acquired surface at " << static_cast<void*>(&surface);
      return &surface;
    }
  }

  RTC_LOG(LS_WARNING) << "No available surface in pool";
  return nullptr;
}

void VplSurfacePool::ReleaseSurface(mfxFrameSurface1* surface) {
  std::lock_guard<std::mutex> lock(mutex_);
  
  if (!surface) {
    return;
  }

  // サーフェスのロックを解除
  surface->Data.Locked = 0;
  RTC_LOG(LS_VERBOSE) << "Released surface at " << static_cast<void*>(surface);
}

void VplSurfacePool::Clear() {
  std::lock_guard<std::mutex> lock(mutex_);
  
  surface_buffer_.clear();
  surfaces_.clear();
  initialized_ = false;
  use_yuy2_ = false;
  width_ = 0;
  height_ = 0;
  
  RTC_LOG(LS_INFO) << "VplSurfacePool cleared";
}

}  // namespace sora