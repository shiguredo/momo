#ifndef SORA_HWENC_VPL_VPL_FRAME_ALLOCATOR_H_
#define SORA_HWENC_VPL_VPL_FRAME_ALLOCATOR_H_

#include <memory>
#include <vector>

// Intel VPL
#include <vpl/mfxvideo++.h>

// VA-API
#include <va/va.h>

namespace sora {

class VplSession;

// VA-API ベースの DMABUF をサポートする VPL フレームアロケータ
class VplFrameAllocator : public mfxFrameAllocator {
 public:
  VplFrameAllocator();
  ~VplFrameAllocator();

  // VA-API ディスプレイを設定
  mfxStatus Init(VADisplay va_display);

  // DMABUF fd を取得
  int GetDmaBufFd(mfxMemId mid);

  // DMABUF fd のリストを取得
  std::vector<int> GetDmaBufFds();

  // mfxFrameAllocator インターフェース実装
  mfxStatus Alloc(mfxFrameAllocRequest* request,
                  mfxFrameAllocResponse* response);
  mfxStatus Lock(mfxMemId mid, mfxFrameData* ptr);
  mfxStatus Unlock(mfxMemId mid, mfxFrameData* ptr);
  mfxStatus GetHDL(mfxMemId mid, mfxHDL* handle);
  mfxStatus Free(mfxFrameAllocResponse* response);

 private:
  struct Surface {
    VASurfaceID va_surface_id;
    int dmabuf_fd;
    bool locked;
  };

  VADisplay va_display_;
  std::vector<std::unique_ptr<Surface>> surfaces_;
  mfxFrameAllocResponse alloc_response_;
};

}  // namespace sora

#endif  // SORA_HWENC_VPL_VPL_FRAME_ALLOCATOR_H_