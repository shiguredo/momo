#ifndef SORA_HWENC_VPL_VPL_VIDEO_PROCESSOR_H_
#define SORA_HWENC_VPL_VPL_VIDEO_PROCESSOR_H_

#include <memory>
#include <vector>

// Intel VPL
#include <vpl/mfxvideo++.h>

#include "sora/vpl_session.h"

namespace sora {

class VplFrameAllocator;

// Intel VPL VPP (Video Pre-Processing) を使用した GPU ベースのビデオ処理
class VplVideoProcessor {
 public:
  VplVideoProcessor(std::shared_ptr<VplSession> session);
  ~VplVideoProcessor();

  // VPP を初期化（YUY2 → NV12 変換用）
  bool Init(int width, int height, int framerate);

  // フレームアロケータを設定
  bool SetFrameAllocator(VplFrameAllocator* allocator);

  // YUY2 → NV12 変換を実行
  // input: YUY2 サーフェース（DMABUF）
  // output: NV12 サーフェース（DMABUF）
  mfxStatus ProcessFrame(mfxFrameSurface1* input, mfxFrameSurface1** output);

  // 入力サーフェースリストを取得
  std::vector<mfxFrameSurface1>& GetInputSurfaces() { return input_surfaces_; }

  // 出力サーフェースリストを取得
  std::vector<mfxFrameSurface1>& GetOutputSurfaces() {
    return output_surfaces_;
  }

  // 空き入力サーフェースを取得
  mfxFrameSurface1* GetFreeInputSurface();

  // 空き出力サーフェースを取得
  mfxFrameSurface1* GetFreeOutputSurface();

 private:
  std::shared_ptr<VplSession> session_;
  std::unique_ptr<MFXVideoVPP> vpp_;
  VplFrameAllocator* allocator_;

  mfxVideoParam vpp_param_;
  mfxFrameAllocRequest vpp_request_[2];  // [0]: 入力, [1]: 出力

  std::vector<mfxFrameSurface1> input_surfaces_;   // YUY2 サーフェース
  std::vector<mfxFrameSurface1> output_surfaces_;  // NV12 サーフェース

  int width_;
  int height_;
  int framerate_;
};

}  // namespace sora

#endif  // SORA_HWENC_VPL_VPL_VIDEO_PROCESSOR_H_