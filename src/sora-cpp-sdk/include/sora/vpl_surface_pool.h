#ifndef SORA_VPL_SURFACE_POOL_H_
#define SORA_VPL_SURFACE_POOL_H_

#include <vpl/mfxstructures.h>
#include <memory>
#include <vector>
#include <mutex>

namespace sora {

// VPL サーフェスメモリプールを管理するクラス
// V4L2 キャプチャラーとエンコーダー間でサーフェスを共有し、
// メモリコピーを削減するための実装
class VplSurfacePool {
 public:
  static VplSurfacePool& GetInstance() {
    static VplSurfacePool instance;
    return instance;
  }

  // プールを初期化（エンコーダー側から呼ばれる）
  void Initialize(int width, int height, int num_surfaces, bool use_yuy2);

  // 空いているサーフェスを取得（V4L2 キャプチャラー側から呼ばれる）
  mfxFrameSurface1* AcquireSurface();

  // サーフェスを解放（VplBackedNativeBuffer のデストラクタから呼ばれる）
  void ReleaseSurface(mfxFrameSurface1* surface);

  // プールが初期化されているか
  bool IsInitialized() const { return initialized_; }

  // YUY2 フォーマットが有効か
  bool IsYuy2Enabled() const { return use_yuy2_; }

  // 幅と高さを取得
  int GetWidth() const { return width_; }
  int GetHeight() const { return height_; }

  // プールをクリア
  void Clear();

 private:
  VplSurfacePool() = default;
  ~VplSurfacePool() = default;
  VplSurfacePool(const VplSurfacePool&) = delete;
  VplSurfacePool& operator=(const VplSurfacePool&) = delete;

  mutable std::mutex mutex_;
  bool initialized_ = false;
  bool use_yuy2_ = false;
  int width_ = 0;
  int height_ = 0;
  std::vector<uint8_t> surface_buffer_;
  std::vector<mfxFrameSurface1> surfaces_;
};

}  // namespace sora

#endif  // SORA_VPL_SURFACE_POOL_H_