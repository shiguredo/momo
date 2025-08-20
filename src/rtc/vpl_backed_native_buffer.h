#ifndef VPL_BACKED_NATIVE_BUFFER_H_
#define VPL_BACKED_NATIVE_BUFFER_H_

#include "native_buffer.h"
#include <vpl/mfxstructures.h>
#include <memory>
#include <functional>

// VPL サーフェスメモリを直接使用する NativeBuffer
// メモリコピーを削減するための最適化実装
class VplBackedNativeBuffer : public NativeBuffer {
 public:
  // サーフェス解放時のコールバック型
  using SurfaceReleaseCallback = std::function<void(mfxFrameSurface1*)>;

  // VPL サーフェスメモリをバッキングとして使用する NativeBuffer を作成
  static webrtc::scoped_refptr<VplBackedNativeBuffer> Create(
      webrtc::VideoType video_type,
      int width,
      int height,
      mfxFrameSurface1* surface,
      SurfaceReleaseCallback release_callback);

  // サーフェスが VPL バッキングかどうかを判定
  bool IsVplBacked() const { return true; }

  // VPL サーフェスへの直接アクセス
  mfxFrameSurface1* GetVplSurface() { return surface_; }
  const mfxFrameSurface1* GetVplSurface() const { return surface_; }

  // NativeBuffer インターフェースのオーバーライド
  const uint8_t* Data() const override;
  uint8_t* MutableData() override;

 protected:
  VplBackedNativeBuffer(webrtc::VideoType video_type,
                        int width,
                        int height,
                        mfxFrameSurface1* surface,
                        SurfaceReleaseCallback release_callback);
  ~VplBackedNativeBuffer() override;

 private:
  mfxFrameSurface1* surface_;
  SurfaceReleaseCallback release_callback_;
};

#endif  // VPL_BACKED_NATIVE_BUFFER_H_