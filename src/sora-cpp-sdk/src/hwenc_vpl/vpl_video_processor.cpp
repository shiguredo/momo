#include "sora/hwenc_vpl/vpl_video_processor.h"

#include <algorithm>
#include <cstring>

// WebRTC
#include <rtc_base/logging.h>

// Intel VPL
#include <vpl/mfxdefs.h>
#include <vpl/mfxstructures.h>

#include "../vpl_session_impl.h"
#include "sora/hwenc_vpl/vpl_frame_allocator.h"
#include "vpl_utils.h"

namespace sora {

VplVideoProcessor::VplVideoProcessor(std::shared_ptr<VplSession> session)
    : session_(session),
      allocator_(nullptr),
      width_(0),
      height_(0),
      framerate_(30) {}

VplVideoProcessor::~VplVideoProcessor() {
  if (vpp_) {
    vpp_->Close();
  }
}

bool VplVideoProcessor::Init(int width, int height, int framerate) {
  width_ = width;
  height_ = height;
  framerate_ = framerate;

  // VPP オブジェクトを作成
  vpp_ = std::make_unique<MFXVideoVPP>(GetVplSession(session_));

  // VPP パラメータを設定
  memset(&vpp_param_, 0, sizeof(vpp_param_));
  vpp_param_.IOPattern =
      MFX_IOPATTERN_IN_VIDEO_MEMORY | MFX_IOPATTERN_OUT_VIDEO_MEMORY;

  // 入力フォーマット（YUY2）
  vpp_param_.vpp.In.FourCC = MFX_FOURCC_YUY2;
  vpp_param_.vpp.In.ChromaFormat = MFX_CHROMAFORMAT_YUV422;
  vpp_param_.vpp.In.Width = (width + 15) / 16 * 16;  // 16 の倍数にアライメント
  vpp_param_.vpp.In.Height = (height + 15) / 16 * 16;
  vpp_param_.vpp.In.CropW = width;
  vpp_param_.vpp.In.CropH = height;
  vpp_param_.vpp.In.FrameRateExtN = framerate;
  vpp_param_.vpp.In.FrameRateExtD = 1;
  vpp_param_.vpp.In.PicStruct = MFX_PICSTRUCT_PROGRESSIVE;

  // 出力フォーマット（NV12）
  vpp_param_.vpp.Out.FourCC = MFX_FOURCC_NV12;
  vpp_param_.vpp.Out.ChromaFormat = MFX_CHROMAFORMAT_YUV420;
  vpp_param_.vpp.Out.Width = (width + 15) / 16 * 16;
  vpp_param_.vpp.Out.Height = (height + 15) / 16 * 16;
  vpp_param_.vpp.Out.CropW = width;
  vpp_param_.vpp.Out.CropH = height;
  vpp_param_.vpp.Out.FrameRateExtN = framerate;
  vpp_param_.vpp.Out.FrameRateExtD = 1;
  vpp_param_.vpp.Out.PicStruct = MFX_PICSTRUCT_PROGRESSIVE;

  // VPP Query を実行して最適なパラメータを取得
  mfxStatus sts = vpp_->Query(&vpp_param_, &vpp_param_);
  if (sts < 0) {
    RTC_LOG(LS_ERROR) << "VPP Query failed: " << sts;
    return false;
  }

  // 必要なサーフェース数を問い合わせ
  memset(vpp_request_, 0, sizeof(vpp_request_));
  sts = vpp_->QueryIOSurf(&vpp_param_, vpp_request_);
  if (sts != MFX_ERR_NONE) {
    RTC_LOG(LS_ERROR) << "VPP QueryIOSurf failed: " << sts;
    return false;
  }

  RTC_LOG(LS_INFO) << "VPP Input surfaces needed: "
                   << vpp_request_[0].NumFrameSuggested
                   << ", Output surfaces needed: "
                   << vpp_request_[1].NumFrameSuggested;

  // VPP を初期化
  sts = vpp_->Init(&vpp_param_);
  if (sts != MFX_ERR_NONE) {
    RTC_LOG(LS_ERROR) << "VPP Init failed: " << sts;
    return false;
  }

  RTC_LOG(LS_INFO) << "VPP initialized for YUY2 -> NV12 conversion at " << width
                   << "x" << height << "@" << framerate << "fps";

  return true;
}

bool VplVideoProcessor::SetFrameAllocator(VplFrameAllocator* allocator) {
  if (!allocator || !vpp_) {
    return false;
  }

  allocator_ = allocator;

  // VPP にフレームアロケータを設定
  mfxStatus sts = vpp_->SetFrameAllocator(allocator_);
  if (sts != MFX_ERR_NONE) {
    RTC_LOG(LS_ERROR) << "Failed to set frame allocator to VPP: " << sts;
    return false;
  }

  // 入力サーフェース（YUY2）を作成
  mfxFrameAllocRequest input_request;
  memset(&input_request, 0, sizeof(input_request));
  input_request.Info = vpp_param_.vpp.In;
  input_request.NumFrameSuggested = vpp_request_[0].NumFrameSuggested;
  input_request.Type = MFX_MEMTYPE_VIDEO_MEMORY_PROCESSOR_TARGET;

  mfxFrameAllocResponse input_response;
  sts = allocator_->Alloc(&input_request, &input_response);
  if (sts != MFX_ERR_NONE) {
    RTC_LOG(LS_ERROR) << "Failed to allocate input surfaces: " << sts;
    return false;
  }

  // 入力サーフェースリストを作成
  input_surfaces_.clear();
  input_surfaces_.reserve(input_response.NumFrameActual);
  for (int i = 0; i < input_response.NumFrameActual; i++) {
    mfxFrameSurface1 surface;
    memset(&surface, 0, sizeof(surface));
    surface.Info = vpp_param_.vpp.In;
    surface.Data.MemId = input_response.mids[i];
    input_surfaces_.push_back(surface);
  }

  // 出力サーフェース（NV12）を作成
  mfxFrameAllocRequest output_request;
  memset(&output_request, 0, sizeof(output_request));
  output_request.Info = vpp_param_.vpp.Out;
  output_request.NumFrameSuggested = vpp_request_[1].NumFrameSuggested;
  output_request.Type = MFX_MEMTYPE_VIDEO_MEMORY_PROCESSOR_TARGET;

  mfxFrameAllocResponse output_response;
  sts = allocator_->Alloc(&output_request, &output_response);
  if (sts != MFX_ERR_NONE) {
    RTC_LOG(LS_ERROR) << "Failed to allocate output surfaces: " << sts;
    return false;
  }

  // 出力サーフェースリストを作成
  output_surfaces_.clear();
  output_surfaces_.reserve(output_response.NumFrameActual);
  for (int i = 0; i < output_response.NumFrameActual; i++) {
    mfxFrameSurface1 surface;
    memset(&surface, 0, sizeof(surface));
    surface.Info = vpp_param_.vpp.Out;
    surface.Data.MemId = output_response.mids[i];
    output_surfaces_.push_back(surface);
  }

  RTC_LOG(LS_INFO) << "VPP surfaces allocated: " << input_surfaces_.size()
                   << " input (YUY2), " << output_surfaces_.size()
                   << " output (NV12)";

  return true;
}

mfxFrameSurface1* VplVideoProcessor::GetFreeInputSurface() {
  auto it =
      std::find_if(input_surfaces_.begin(), input_surfaces_.end(),
                   [](const mfxFrameSurface1& s) { return !s.Data.Locked; });
  if (it != input_surfaces_.end()) {
    return &(*it);
  }
  return nullptr;
}

mfxFrameSurface1* VplVideoProcessor::GetFreeOutputSurface() {
  auto it =
      std::find_if(output_surfaces_.begin(), output_surfaces_.end(),
                   [](const mfxFrameSurface1& s) { return !s.Data.Locked; });
  if (it != output_surfaces_.end()) {
    return &(*it);
  }
  return nullptr;
}

mfxStatus VplVideoProcessor::ProcessFrame(mfxFrameSurface1* input,
                                          mfxFrameSurface1** output) {
  if (!vpp_ || !input) {
    return MFX_ERR_NULL_PTR;
  }

  // 空き出力サーフェースを取得
  mfxFrameSurface1* out_surface = GetFreeOutputSurface();
  if (!out_surface) {
    RTC_LOG(LS_WARNING) << "No free output surface available";
    return MFX_ERR_NOT_ENOUGH_BUFFER;
  }

  mfxSyncPoint syncp;
  mfxStatus sts = MFX_ERR_NONE;

  // VPP で YUY2 → NV12 変換を実行
  for (;;) {
    sts = vpp_->RunFrameVPPAsync(input, out_surface, nullptr, &syncp);

    if (sts == MFX_WRN_DEVICE_BUSY) {
      // GPU が忙しい場合は少し待つ
      usleep(1000);
      continue;
    }
    break;
  }

  if (sts < 0) {
    RTC_LOG(LS_ERROR) << "VPP RunFrameVPPAsync failed: " << sts;
    return sts;
  }

  // 非同期処理の完了を待つ
  if (syncp) {
    sts = MFXVideoCORE_SyncOperation(GetVplSession(session_), syncp, 60000);
    if (sts != MFX_ERR_NONE) {
      RTC_LOG(LS_ERROR) << "VPP SyncOperation failed: " << sts;
      return sts;
    }
  }

  *output = out_surface;
  return MFX_ERR_NONE;
}

}  // namespace sora