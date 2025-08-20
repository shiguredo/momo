#include "vpl_backed_native_buffer.h"
#include <rtc_base/checks.h>
#include <rtc_base/logging.h>

webrtc::scoped_refptr<VplBackedNativeBuffer> VplBackedNativeBuffer::Create(
    webrtc::VideoType video_type,
    int width,
    int height,
    mfxFrameSurface1* surface,
    SurfaceReleaseCallback release_callback) {
  RTC_CHECK(surface);
  RTC_CHECK(surface->Data.Y);
  return webrtc::make_ref_counted<VplBackedNativeBuffer>(
      video_type, width, height, surface, release_callback);
}

const uint8_t* VplBackedNativeBuffer::Data() const {
  // YUY2 の場合、パックドフォーマットなので Y ポインタがデータ全体を指す
  return surface_->Data.Y;
}

uint8_t* VplBackedNativeBuffer::MutableData() {
  return surface_->Data.Y;
}

VplBackedNativeBuffer::VplBackedNativeBuffer(
    webrtc::VideoType video_type,
    int width,
    int height,
    mfxFrameSurface1* surface,
    SurfaceReleaseCallback release_callback)
    : NativeBuffer(video_type, width, height),
      surface_(surface),
      release_callback_(release_callback) {
  // サーフェスをロック
  surface_->Data.Locked = 1;

  // YUY2 の場合のデータサイズを設定
  if (video_type == webrtc::VideoType::kYUY2) {
    SetLength(width * height * 2);
  }

  RTC_LOG(LS_VERBOSE) << "VplBackedNativeBuffer created for " << width << "x"
                      << height << " surface at "
                      << static_cast<void*>(surface);
}

VplBackedNativeBuffer::~VplBackedNativeBuffer() {
  // サーフェスのロックを解除してプールに返却
  if (surface_) {
    surface_->Data.Locked = 0;
    if (release_callback_) {
      release_callback_(surface_);
    }
    RTC_LOG(LS_VERBOSE) << "VplBackedNativeBuffer released surface at "
                        << static_cast<void*>(surface_);
  }
}