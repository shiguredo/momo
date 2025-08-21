#include "sora/v4l2_vpl_capturer.h"

// WebRTC
#include <rtc_base/logging.h>

namespace sora {

webrtc::scoped_refptr<V4L2VideoCapturer> V4L2VplCapturer::CreateWithVplEncoder(
    const V4L2VideoCapturerConfig& config,
    std::shared_ptr<VplVideoEncoder> encoder) {
  // VPL エンコーダーから DMABUF fd を取得
  std::vector<int> dmabuf_fds = encoder->GetDmaBufFds();

  if (dmabuf_fds.empty()) {
    RTC_LOG(LS_WARNING) << "Failed to get DMABUF fds from VPL encoder";
    // 通常のキャプチャモードにフォールバック
    return V4L2VideoCapturer::Create(config);
  }

  // DMABUF モードで V4L2 キャプチャを設定
  V4L2VideoCapturerConfig dmabuf_config = config;
  dmabuf_config.use_dmabuf = true;
  dmabuf_config.dmabuf_fds = dmabuf_fds;
  dmabuf_config.force_yuy2 = true;  // YUY2 形式を強制

  // V4L2 キャプチャを作成
  auto capturer = V4L2VideoCapturer::Create(dmabuf_config);
  if (!capturer) {
    RTC_LOG(LS_ERROR) << "Failed to create V4L2 capturer with DMABUF";
    return nullptr;
  }

  // DMABUF コールバックを設定
  // V4L2 がバッファをキャプチャしたら、VPL エンコーダーに通知
  capturer->SetDmaBufCallback([encoder](int buffer_index) {
    // VPP で YUY2 -> NV12 変換後、エンコード
    // ここでは dynamic_cast を使用して具象クラスのメソッドを呼び出す
    // または、VplVideoEncoder インターフェースに仮想関数を追加
    // encoder->OnVppSurfaceReady(buffer_index);
    RTC_LOG(LS_VERBOSE) << "DMABUF buffer " << buffer_index
                        << " ready for VPP processing";
  });

  RTC_LOG(LS_INFO) << "Created V4L2 capturer with " << dmabuf_fds.size()
                   << " DMABUF surfaces for VPL zero-copy pipeline";

  return capturer;
}

std::vector<int> V4L2VplCapturer::GetDmaBufFdsFromEncoder(
    std::shared_ptr<VplVideoEncoder> encoder) {
  // VPL エンコーダーから DMABUF fd を取得
  return encoder->GetDmaBufFds();
}

}  // namespace sora