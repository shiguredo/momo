#include "sora/hwenc_vpl/vpl_video_encoder.h"

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <mutex>
#include <vector>

// WebRTC
#include <api/scoped_refptr.h>
#include <api/video/encoded_image.h>
#include <api/video/render_resolution.h>
#include <api/video/video_codec_type.h>
#include <api/video/video_content_type.h>
#include <api/video/video_frame.h>
#include <api/video/video_frame_buffer.h>
#include <api/video/video_frame_type.h>
#include <api/video/video_timing.h>
#include <api/video_codecs/scalability_mode.h>
#include <api/video_codecs/video_codec.h>
#include <api/video_codecs/video_encoder.h>
#include <common_video/h264/h264_bitstream_parser.h>
#include <common_video/h265/h265_bitstream_parser.h>
#include <common_video/include/bitrate_adjuster.h>
#include <modules/video_coding/codecs/h264/include/h264_globals.h>
#include <modules/video_coding/codecs/interface/common_constants.h>
#include <modules/video_coding/codecs/vp9/include/vp9_globals.h>
#include <modules/video_coding/include/video_codec_interface.h>
#include <modules/video_coding/include/video_error_codes.h>
#include <modules/video_coding/svc/create_scalability_structure.h>
#include <modules/video_coding/svc/scalable_video_controller.h>
#include <modules/video_coding/utility/vp9_uncompressed_header_parser.h>
#include <rtc_base/checks.h>
#include <rtc_base/logging.h>

// libyuv
#include <libyuv/convert_from.h>
#include <libyuv/planar_functions.h>

// Intel VPL
#include <vpl/mfxcommon.h>
#include <vpl/mfxdefs.h>
#include <vpl/mfxstructures.h>
#include <vpl/mfxvideo++.h>
#include <vpl/mfxvideo.h>
#include <vpl/mfxvp8.h>

#include "../vpl_session_impl.h"
#include "sora/vpl_session.h"
#include "vpl_utils.h"

// VA-API for DMABUF support
#ifdef __linux__
#include <fcntl.h>
#include <unistd.h>
#include <va/va.h>
#include <va/va_drm.h>
#include "sora/hwenc_vpl/vpl_frame_allocator.h"
#include "sora/hwenc_vpl/vpl_video_processor.h"
#endif

namespace sora {

class VplVideoEncoderImpl : public VplVideoEncoder {
 public:
  VplVideoEncoderImpl(std::shared_ptr<VplSession> session, mfxU32 codec);
  ~VplVideoEncoderImpl() override;

  int32_t InitEncode(const webrtc::VideoCodec* codec_settings,
                     int32_t number_of_cores,
                     size_t max_payload_size) override;
  int32_t RegisterEncodeCompleteCallback(
      webrtc::EncodedImageCallback* callback) override;
  int32_t Release() override;
  int32_t Encode(
      const webrtc::VideoFrame& frame,
      const std::vector<webrtc::VideoFrameType>* frame_types) override;
  void SetRates(const RateControlParameters& parameters) override;
  webrtc::VideoEncoder::EncoderInfo GetEncoderInfo() const override;

  // DMABUF サポート
  std::vector<int> GetDmaBufFds() const override;
  bool EnableDmaBufMode() override;

  static std::unique_ptr<MFXVideoENCODE> CreateEncoder(
      std::shared_ptr<VplSession> session,
      mfxU32 codec,
      int width,
      int height,
      int framerate,
      int target_kbps,
      int max_kbps,
      bool init);

 private:
  struct ExtBuffer {
    mfxExtBuffer* ext_buffers[10];
    mfxExtCodingOption ext_coding_option;
    mfxExtCodingOption2 ext_coding_option2;
  };
  // いろいろなパターンでクエリを投げて、
  // 成功した時の param を返す
  static mfxStatus Queries(MFXVideoENCODE* encoder,
                           mfxU32 codec,
                           int width,
                           int height,
                           int framerate,
                           int target_kbps,
                           int max_kbps,
                           mfxVideoParam& param,
                           ExtBuffer& ext);

 private:
  std::mutex mutex_;
  webrtc::EncodedImageCallback* callback_ = nullptr;
  webrtc::BitrateAdjuster bitrate_adjuster_;
  uint32_t target_bitrate_bps_ = 0;
  uint32_t max_bitrate_bps_ = 0;
  bool reconfigure_needed_ = false;
  bool use_native_ = false;
  uint32_t width_ = 0;
  uint32_t height_ = 0;
  uint32_t framerate_ = 0;
  webrtc::VideoCodecMode mode_ = webrtc::VideoCodecMode::kRealtimeVideo;
  std::vector<std::vector<uint8_t>> v_packet_;
  webrtc::EncodedImage encoded_image_;
  webrtc::H264BitstreamParser h264_bitstream_parser_;
  webrtc::H265BitstreamParser h265_bitstream_parser_;
  webrtc::GofInfoVP9 gof_;
  size_t gof_idx_;

  // AV1 用
  std::unique_ptr<webrtc::ScalableVideoController> svc_controller_;
  webrtc::ScalabilityMode scalability_mode_;

  int32_t InitVpl();
  int32_t ReleaseVpl();

  std::vector<uint8_t> surface_buffer_;
  std::vector<mfxFrameSurface1> surfaces_;

#ifdef __linux__
  // DMABUF サポート用
  std::unique_ptr<VplFrameAllocator> frame_allocator_;
  std::unique_ptr<VplVideoProcessor> vpp_;  // VPP プロセッサ
  VADisplay va_display_ = nullptr;
  bool use_dmabuf_ = false;
  std::vector<int> dmabuf_fds_;
  int current_vpp_surface_index_ =
      0;  // 現在処理中の VPP サーフェースインデックス
#endif

  std::shared_ptr<VplSession> session_;
  mfxU32 codec_;
  mfxFrameAllocRequest alloc_request_;
  std::unique_ptr<MFXVideoENCODE> encoder_;
  std::vector<uint8_t> bitstream_buffer_;
  mfxBitstream bitstream_;
  mfxFrameInfo frame_info_;

  int key_frame_interval_ = 0;

  // VPP サーフェースが準備できたときの処理
  void OnVppSurfaceReady(int surface_index);
};

const int kLowH264QpThreshold = 34;
const int kHighH264QpThreshold = 40;

VplVideoEncoderImpl::VplVideoEncoderImpl(std::shared_ptr<VplSession> session,
                                         mfxU32 codec)
    : session_(session), codec_(codec), bitrate_adjuster_(0.5, 0.95) {}

VplVideoEncoderImpl::~VplVideoEncoderImpl() {
  Release();
}

std::unique_ptr<MFXVideoENCODE> VplVideoEncoderImpl::CreateEncoder(
    std::shared_ptr<VplSession> session,
    mfxU32 codec,
    int width,
    int height,
    int framerate,
    int target_kbps,
    int max_kbps,
    bool init) {
  std::unique_ptr<MFXVideoENCODE> encoder(
      new MFXVideoENCODE(GetVplSession(session)));

  // mfxPlatform platform;
  // memset(&platform, 0, sizeof(platform));
  // MFXVideoCORE_QueryPlatform(GetVplSession(session), &platform);
  // RTC_LOG(LS_VERBOSE) << "--------------- codec=" << CodecToString(codec)
  //                     << " CodeName=" << platform.CodeName
  //                     << " DeviceId=" << platform.DeviceId
  //                     << " MediaAdapterType=" << platform.MediaAdapterType;

  mfxVideoParam param;
  ExtBuffer ext;
  mfxStatus sts = Queries(encoder.get(), codec, width, height, framerate,
                          target_kbps, max_kbps, param, ext);
  if (sts < MFX_ERR_NONE) {
    return nullptr;
  }
  if (sts > MFX_ERR_NONE) {
    RTC_LOG(LS_VERBOSE) << "Supported specified codec but has warning: codec="
                        << CodecToString(codec) << " sts=" << sts;
  }

  if (init) {
    sts = encoder->Init(&param);
    if (sts != MFX_ERR_NONE) {
      RTC_LOG(LS_ERROR) << "Failed to Init: sts=" << sts;
      return nullptr;
    }
  }

  return encoder;
}

mfxStatus VplVideoEncoderImpl::Queries(MFXVideoENCODE* encoder,
                                       mfxU32 codec,
                                       int width,
                                       int height,
                                       int framerate,
                                       int target_kbps,
                                       int max_kbps,
                                       mfxVideoParam& param,
                                       ExtBuffer& ext) {
  mfxStatus sts = MFX_ERR_NONE;

  memset(&param, 0, sizeof(param));

  param.mfx.CodecId = codec;
  if (codec == MFX_CODEC_VP8) {
    //param.mfx.CodecProfile = MFX_PROFILE_VP8_0;
  } else if (codec == MFX_CODEC_VP9) {
    //param.mfx.CodecProfile = MFX_PROFILE_VP9_0;
  } else if (codec == MFX_CODEC_AVC) {
    //param.mfx.CodecProfile = MFX_PROFILE_AVC_HIGH;
    //param.mfx.CodecLevel = MFX_LEVEL_AVC_51;
    //param.mfx.CodecProfile = MFX_PROFILE_AVC_MAIN;
    //param.mfx.CodecLevel = MFX_LEVEL_AVC_1;
  } else if (codec == MFX_CODEC_HEVC) {
    // param.mfx.CodecProfile = MFX_PROFILE_HEVC_MAIN;
    // param.mfx.CodecLevel = MFX_LEVEL_HEVC_1;
    // param.mfx.LowPower = MFX_CODINGOPTION_OFF;
  } else if (codec == MFX_CODEC_AV1) {
    //param.mfx.CodecProfile = MFX_PROFILE_AV1_MAIN;
  }

  param.mfx.TargetUsage = MFX_TARGETUSAGE_BALANCED;

  param.mfx.TargetKbps = target_kbps;
  param.mfx.MaxKbps = max_kbps;
  param.mfx.RateControlMethod = MFX_RATECONTROL_VBR;
  param.mfx.FrameInfo.FrameRateExtN = framerate;
  param.mfx.FrameInfo.FrameRateExtD = 1;
  param.mfx.FrameInfo.FourCC = MFX_FOURCC_NV12;
  param.mfx.FrameInfo.ChromaFormat = MFX_CHROMAFORMAT_YUV420;
  param.mfx.FrameInfo.PicStruct = MFX_PICSTRUCT_PROGRESSIVE;
  param.mfx.FrameInfo.CropX = 0;
  param.mfx.FrameInfo.CropY = 0;
  param.mfx.FrameInfo.CropW = width;
  param.mfx.FrameInfo.CropH = height;
  // Width must be a multiple of 16
  // Height must be a multiple of 16 in case of frame picture and a multiple of 32 in case of field picture
  param.mfx.FrameInfo.Width = (width + 15) / 16 * 16;
  param.mfx.FrameInfo.Height = (height + 15) / 16 * 16;

  param.mfx.GopRefDist = 1;
  param.AsyncDepth = 1;
  param.IOPattern =
      MFX_IOPATTERN_IN_SYSTEM_MEMORY | MFX_IOPATTERN_OUT_SYSTEM_MEMORY;

  mfxExtBuffer** ext_buffers = ext.ext_buffers;
  mfxExtCodingOption& ext_coding_option = ext.ext_coding_option;
  mfxExtCodingOption2& ext_coding_option2 = ext.ext_coding_option2;
  int ext_buffers_size = 0;
  if (codec == MFX_CODEC_AVC) {
    memset(&ext_coding_option, 0, sizeof(ext_coding_option));
    ext_coding_option.Header.BufferId = MFX_EXTBUFF_CODING_OPTION;
    ext_coding_option.Header.BufferSz = sizeof(ext_coding_option);
    ext_coding_option.AUDelimiter = MFX_CODINGOPTION_OFF;
    ext_coding_option.MaxDecFrameBuffering = 1;
    //ext_coding_option.NalHrdConformance = MFX_CODINGOPTION_OFF;
    //ext_coding_option.VuiVclHrdParameters = MFX_CODINGOPTION_ON;
    //ext_coding_option.SingleSeiNalUnit = MFX_CODINGOPTION_ON;
    //ext_coding_option.RefPicMarkRep = MFX_CODINGOPTION_OFF;
    //ext_coding_option.PicTimingSEI = MFX_CODINGOPTION_OFF;
    //ext_coding_option.RecoveryPointSEI = MFX_CODINGOPTION_OFF;
    //ext_coding_option.FramePicture = MFX_CODINGOPTION_OFF;
    //ext_coding_option.FieldOutput = MFX_CODINGOPTION_ON;

    memset(&ext_coding_option2, 0, sizeof(ext_coding_option2));
    ext_coding_option2.Header.BufferId = MFX_EXTBUFF_CODING_OPTION2;
    ext_coding_option2.Header.BufferSz = sizeof(ext_coding_option2);
    ext_coding_option2.RepeatPPS = MFX_CODINGOPTION_ON;
    //ext_coding_option2.MaxSliceSize = 1;
    //ext_coding_option2.AdaptiveI = MFX_CODINGOPTION_ON;

    ext_buffers[0] = (mfxExtBuffer*)&ext_coding_option;
    ext_buffers[1] = (mfxExtBuffer*)&ext_coding_option2;
    ext_buffers_size = 2;
  } else if (codec == MFX_CODEC_HEVC) {
    memset(&ext_coding_option2, 0, sizeof(ext_coding_option2));
    ext_coding_option2.Header.BufferId = MFX_EXTBUFF_CODING_OPTION2;
    ext_coding_option2.Header.BufferSz = sizeof(ext_coding_option2);
    ext_coding_option2.RepeatPPS = MFX_CODINGOPTION_ON;

    ext_buffers[0] = (mfxExtBuffer*)&ext_coding_option2;
    ext_buffers_size = 1;
  }

  if (ext_buffers_size != 0) {
    param.ExtParam = ext_buffers;
    param.NumExtParam = ext_buffers_size;
  }

  // Query 関数を呼び出す。
  // 失敗した場合 param は一切書き換わらない
  // 成功した場合 param は書き換わる可能性がある
  auto query = [](MFXVideoENCODE* encoder, mfxVideoParam& param) {
    mfxVideoParam query_param;
    memcpy(&query_param, &param, sizeof(param));
    // ドキュメントによると、Query は以下のエラーを返す可能性がある。
    // MFX_ERR_NONE	The function completed successfully.
    // MFX_ERR_UNSUPPORTED	The function failed to identify a specific implementation for the required features.
    // MFX_WRN_PARTIAL_ACCELERATION	The underlying hardware does not fully support the specified video parameters; The encoding may be partially accelerated. Only SDK HW implementations may return this status code.
    // MFX_WRN_INCOMPATIBLE_VIDEO_PARAM	The function detected some video parameters were incompatible with others; incompatibility resolved.
    mfxStatus sts = encoder->Query(&query_param, &query_param);
    if (sts >= 0) {
      // デバッグ用。
      // Query によってどのパラメータが変更されたかを表示する
      // #define F(NAME)                                           \
      //   if (param.NAME != query_param.NAME)                     \
      //   std::cout << "param " << #NAME << " old=" << param.NAME \
      //             << " new=" << query_param.NAME << std::endl
      //       F(mfx.LowPower);
      //       F(mfx.BRCParamMultiplier);
      //       F(mfx.FrameInfo.FrameRateExtN);
      //       F(mfx.FrameInfo.FrameRateExtD);
      //       F(mfx.FrameInfo.FourCC);
      //       F(mfx.FrameInfo.ChromaFormat);
      //       F(mfx.FrameInfo.PicStruct);
      //       F(mfx.FrameInfo.CropX);
      //       F(mfx.FrameInfo.CropY);
      //       F(mfx.FrameInfo.CropW);
      //       F(mfx.FrameInfo.CropH);
      //       F(mfx.FrameInfo.Width);
      //       F(mfx.FrameInfo.Height);
      //       F(mfx.CodecId);
      //       F(mfx.CodecProfile);
      //       F(mfx.CodecLevel);
      //       F(mfx.GopPicSize);
      //       F(mfx.GopRefDist);
      //       F(mfx.GopOptFlag);
      //       F(mfx.IdrInterval);
      //       F(mfx.TargetUsage);
      //       F(mfx.RateControlMethod);
      //       F(mfx.InitialDelayInKB);
      //       F(mfx.TargetKbps);
      //       F(mfx.MaxKbps);
      //       F(mfx.BufferSizeInKB);
      //       F(mfx.NumSlice);
      //       F(mfx.NumRefFrame);
      //       F(mfx.EncodedOrder);
      //       F(mfx.DecodedOrder);
      //       F(mfx.ExtendedPicStruct);
      //       F(mfx.TimeStampCalc);
      //       F(mfx.SliceGroupsPresent);
      //       F(mfx.MaxDecFrameBuffering);
      //       F(mfx.EnableReallocRequest);
      //       F(AsyncDepth);
      //       F(IOPattern);
      // #undef F

      memcpy(&param, &query_param, sizeof(param));
    }
    return sts;
  };

  // ここからは、ひたすらパラメータを変えて query を呼び出していく
  sts = query(encoder, param);
  if (sts >= 0) {
    return sts;
  }

  // IOPattern を MFX_IOPATTERN_IN_SYSTEM_MEMORY のみにしてみる
  // Coffee Lake の H265 はこのパターンでないと通らない
  RTC_LOG(LS_VERBOSE) << "Unsupported encoder codec: codec="
                      << CodecToString(codec) << " sts=" << sts
                      << " ... Retry with IOPattern IN_SYSTEM_MEMORY only";
  param.IOPattern = MFX_IOPATTERN_IN_SYSTEM_MEMORY;
  sts = query(encoder, param);
  if (sts >= 0) {
    return sts;
  }

  // LowPower ON にして、更に H264/H265 は固定 QP モードにしてみる
  RTC_LOG(LS_VERBOSE) << "Unsupported encoder codec: codec="
                      << CodecToString(codec) << " sts=" << sts
                      << " ... Retry with low power mode";
  param.mfx.LowPower = MFX_CODINGOPTION_ON;
  if (codec == MFX_CODEC_AVC || codec == MFX_CODEC_HEVC) {
    param.mfx.RateControlMethod = MFX_RATECONTROL_CQP;
    param.mfx.QPI = 25;
    param.mfx.QPP = 33;
    param.mfx.QPB = 40;
  }
  sts = query(encoder, param);
  if (sts >= 0) {
    return sts;
  }
  RTC_LOG(LS_VERBOSE) << "Unsupported encoder codec: codec="
                      << CodecToString(codec) << " sts=" << sts;

  return sts;
}

int32_t VplVideoEncoderImpl::InitEncode(
    const webrtc::VideoCodec* codec_settings,
    int32_t number_of_cores,
    size_t max_payload_size) {
  RTC_DCHECK(codec_settings);

  int32_t release_ret = Release();
  if (release_ret != WEBRTC_VIDEO_CODEC_OK) {
    return release_ret;
  }

  width_ = codec_settings->width;
  height_ = codec_settings->height;
  target_bitrate_bps_ = codec_settings->startBitrate * 1000;
  max_bitrate_bps_ = codec_settings->maxBitrate * 1000;
  bitrate_adjuster_.SetTargetBitrateBps(target_bitrate_bps_);
  framerate_ = codec_settings->maxFramerate;
  mode_ = codec_settings->mode;

  RTC_LOG(LS_INFO) << "InitEncode " << target_bitrate_bps_ << "bit/sec";

  // Initialize encoded image. Default buffer size: size of unencoded data.
  encoded_image_._encodedWidth = 0;
  encoded_image_._encodedHeight = 0;
  encoded_image_.set_size(0);
  encoded_image_.timing_.flags =
      webrtc::VideoSendTiming::TimingFrameFlags::kInvalid;
  encoded_image_.content_type_ =
      (codec_settings->mode == webrtc::VideoCodecMode::kScreensharing)
          ? webrtc::VideoContentType::SCREENSHARE
          : webrtc::VideoContentType::UNSPECIFIED;

  if (codec_ == MFX_CODEC_VP9) {
    gof_.SetGofInfoVP9(webrtc::TemporalStructureMode::kTemporalStructureMode1);
    gof_idx_ = 0;
  }

  if (codec_ == MFX_CODEC_AV1) {
    auto scalability_mode = codec_settings->GetScalabilityMode();
    if (!scalability_mode) {
      RTC_LOG(LS_WARNING) << "Scalability mode is not set, using 'L1T1'.";
      scalability_mode = webrtc::ScalabilityMode::kL1T1;
    }
    RTC_LOG(LS_INFO) << "InitEncode scalability_mode:"
                     << (int)*scalability_mode;
    svc_controller_ = webrtc::CreateScalabilityStructure(*scalability_mode);
    scalability_mode_ = *scalability_mode;
  }

  return InitVpl();
}
int32_t VplVideoEncoderImpl::RegisterEncodeCompleteCallback(
    webrtc::EncodedImageCallback* callback) {
  std::lock_guard<std::mutex> lock(mutex_);
  callback_ = callback;
  return WEBRTC_VIDEO_CODEC_OK;
}
int32_t VplVideoEncoderImpl::Release() {
  return ReleaseVpl();
}
int32_t VplVideoEncoderImpl::Encode(
    const webrtc::VideoFrame& frame,
    const std::vector<webrtc::VideoFrameType>* frame_types) {
  bool send_key_frame = false;

  if (frame_types != nullptr) {
    // We only support a single stream.
    RTC_DCHECK_EQ(frame_types->size(), static_cast<size_t>(1));
    // Skip frame?
    if ((*frame_types)[0] == webrtc::VideoFrameType::kEmptyFrame) {
      return WEBRTC_VIDEO_CODEC_OK;
    }
    // Force key frame?
    send_key_frame =
        (*frame_types)[0] == webrtc::VideoFrameType::kVideoFrameKey;
  }

  // 使ってない入力サーフェスを取り出す
  auto surface =
      std::find_if(surfaces_.begin(), surfaces_.end(),
                   [](const mfxFrameSurface1& s) { return !s.Data.Locked; });
  if (surface == surfaces_.end()) {
    RTC_LOG(LS_ERROR) << "Surface not found";
    return WEBRTC_VIDEO_CODEC_ERROR;
  }

  // フレームバッファのタイプをチェック
  auto video_frame_buffer = frame.video_frame_buffer();

  if (video_frame_buffer->type() == webrtc::VideoFrameBuffer::Type::kNV12) {
    // NV12 の場合はそのまま利用する
    auto nv12_buffer = video_frame_buffer->GetNV12();
    libyuv::NV12Copy(nv12_buffer->DataY(), nv12_buffer->StrideY(),
                     nv12_buffer->DataUV(), nv12_buffer->StrideUV(),
                     surface->Data.Y, surface->Data.Pitch, surface->Data.U,
                     surface->Data.Pitch, frame.width(), frame.height());
  } else {
    // それ以外のフレームバッファは I420 経由で変換
    webrtc::scoped_refptr<const webrtc::I420BufferInterface> frame_buffer =
        video_frame_buffer->ToI420();
    libyuv::I420ToNV12(
        frame_buffer->DataY(), frame_buffer->StrideY(), frame_buffer->DataU(),
        frame_buffer->StrideU(), frame_buffer->DataV(), frame_buffer->StrideV(),
        surface->Data.Y, surface->Data.Pitch, surface->Data.U,
        surface->Data.Pitch, frame_buffer->width(), frame_buffer->height());
  }

  mfxStatus sts;

  mfxEncodeCtrl ctrl;
  memset(&ctrl, 0, sizeof(ctrl));
  if (send_key_frame) {
    // VP9 では MFX_FRAMETYPE_I のみを設定する
    // VP9 は MFX_FRAMETYPE_I か MFX_FRAMETYPE_P のみのフレームを想定しているようで、
    // 他のフレームタイプを設定すると内部的に MFX_FRAMETYPE_P に変更されてしまう
    // ref: https://github.com/intel/vpl-gpu-rt/blob/5d99334834aafc5448e7c799a7c176ee0832ec09/_studio/mfx_lib/encode_hw/vp9/src/mfx_vp9_encode_hw_par.cpp#L1853-L1857
    if (codec_ == MFX_CODEC_VP9) {
      ctrl.FrameType = MFX_FRAMETYPE_I;
    } else {
      ctrl.FrameType = MFX_FRAMETYPE_I | MFX_FRAMETYPE_IDR | MFX_FRAMETYPE_REF;
    }
  } else {
    ctrl.FrameType = MFX_FRAMETYPE_UNKNOWN;
  }

  if (reconfigure_needed_) {
    auto start_time = std::chrono::system_clock::now();
    RTC_LOG(LS_INFO) << "Start reconfigure: bps="
                     << (bitrate_adjuster_.GetAdjustedBitrateBps() / 1000)
                     << " framerate=" << framerate_;
    // 今の設定を取得する
    mfxVideoParam param;
    memset(&param, 0, sizeof(param));

    sts = encoder_->GetVideoParam(&param);
    VPL_CHECK_RESULT(sts, MFX_ERR_NONE, sts);

    // ビットレートとフレームレートを変更する。
    // なお、encoder_->Reset() はキューイングしているサーフェスを
    // 全て処理してから呼び出す必要がある。
    // ここでは encoder_->Init() の時に
    //   param.mfx.GopRefDist = 1;
    //   param.AsyncDepth = 1;
    //   ext_coding_option.MaxDecFrameBuffering = 1;
    // を設定して、そもそもキューイングが起きないようにすることで対処している。
    if (param.mfx.RateControlMethod == MFX_RATECONTROL_CQP) {
      //param.mfx.QPI = h264_bitstream_parser_.GetLastSliceQp().value_or(30);
    } else {
      param.mfx.TargetKbps = bitrate_adjuster_.GetAdjustedBitrateBps() / 1000;
    }
    param.mfx.FrameInfo.FrameRateExtN = framerate_;
    param.mfx.FrameInfo.FrameRateExtD = 1;

    sts = encoder_->Reset(&param);
    VPL_CHECK_RESULT(sts, MFX_ERR_NONE, sts);

    reconfigure_needed_ = false;

    auto end_time = std::chrono::system_clock::now();
    RTC_LOG(LS_INFO) << "Finish reconfigure: "
                     << std::chrono::duration_cast<std::chrono::milliseconds>(
                            end_time - start_time)
                            .count()
                     << " ms";
  }

  // NV12 をハードウェアエンコード
  mfxSyncPoint syncp;
  sts = encoder_->EncodeFrameAsync(&ctrl, &*surface, &bitstream_, &syncp);
  // alloc_request_.NumFrameSuggested が 1 の場合は MFX_ERR_MORE_DATA は発生しない
  if (sts == MFX_ERR_MORE_DATA) {
    // もっと入力が必要なので出直す
    return WEBRTC_VIDEO_CODEC_OK;
  }
  VPL_CHECK_RESULT(sts, MFX_ERR_NONE, sts);

  sts = MFXVideoCORE_SyncOperation(GetVplSession(session_), syncp, 600000);
  VPL_CHECK_RESULT(sts, MFX_ERR_NONE, sts);

  //RTC_LOG(LS_ERROR) << "SurfaceSize=" << (surface->Data.U - surface->Data.Y);
  //RTC_LOG(LS_ERROR) << "DataLength=" << bitstream_.DataLength;
  {
    uint8_t* p = bitstream_.Data + bitstream_.DataOffset;
    int size = bitstream_.DataLength;
    bitstream_.DataLength = 0;

    //FILE* fp = fopen("test.mp4", "a+");
    //fwrite(p, 1, size, fp);
    //fclose(fp);

    if (codec_ == MFX_CODEC_VP9) {
      // VP9 はIVFヘッダーがエンコードフレームについているので取り除く
      if ((p[0] == 'D') && (p[1] == 'K') && (p[2] == 'I') && (p[3] == 'F')) {
        p += 32;
        size -= 32;
      }
      p += 12;
      size -= 12;
    }
    auto buf = webrtc::EncodedImageBuffer::Create(p, size);
    encoded_image_.SetEncodedData(buf);
    encoded_image_._encodedWidth = width_;
    encoded_image_._encodedHeight = height_;
    encoded_image_.content_type_ =
        (mode_ == webrtc::VideoCodecMode::kScreensharing)
            ? webrtc::VideoContentType::SCREENSHARE
            : webrtc::VideoContentType::UNSPECIFIED;
    encoded_image_.timing_.flags = webrtc::VideoSendTiming::kInvalid;
    encoded_image_.SetRtpTimestamp(frame.rtp_timestamp());
    encoded_image_.ntp_time_ms_ = frame.ntp_time_ms();
    encoded_image_.capture_time_ms_ = frame.render_time_ms();
    encoded_image_.rotation_ = frame.rotation();
    encoded_image_.SetColorSpace(frame.color_space());
    key_frame_interval_ += 1;
    if (bitstream_.FrameType & MFX_FRAMETYPE_I ||
        bitstream_.FrameType & MFX_FRAMETYPE_IDR) {
      encoded_image_._frameType = webrtc::VideoFrameType::kVideoFrameKey;
      RTC_LOG(LS_INFO) << "Key Frame Generated: key_frame_interval="
                       << key_frame_interval_;
      key_frame_interval_ = 0;
    } else {
      encoded_image_._frameType = webrtc::VideoFrameType::kVideoFrameDelta;
    }

    webrtc::CodecSpecificInfo codec_specific;
    if (codec_ == MFX_CODEC_VP9) {
      codec_specific.codecType = webrtc::kVideoCodecVP9;
      bool is_key =
          encoded_image_._frameType == webrtc::VideoFrameType::kVideoFrameKey;
      if (is_key) {
        gof_idx_ = 0;
      }
      codec_specific.codecSpecific.VP9.inter_pic_predicted = !is_key;
      codec_specific.codecSpecific.VP9.flexible_mode = false;
      codec_specific.codecSpecific.VP9.ss_data_available = is_key;
      codec_specific.codecSpecific.VP9.temporal_idx = webrtc::kNoTemporalIdx;
      codec_specific.codecSpecific.VP9.temporal_up_switch = true;
      codec_specific.codecSpecific.VP9.inter_layer_predicted = false;
      codec_specific.codecSpecific.VP9.gof_idx =
          static_cast<uint8_t>(gof_idx_++ % gof_.num_frames_in_gof);
      codec_specific.codecSpecific.VP9.num_spatial_layers = 1;
      codec_specific.codecSpecific.VP9.first_frame_in_picture = true;
      codec_specific.codecSpecific.VP9.spatial_layer_resolution_present = false;
      if (codec_specific.codecSpecific.VP9.ss_data_available) {
        codec_specific.codecSpecific.VP9.spatial_layer_resolution_present =
            true;
        codec_specific.codecSpecific.VP9.width[0] =
            encoded_image_._encodedWidth;
        codec_specific.codecSpecific.VP9.height[0] =
            encoded_image_._encodedHeight;
        codec_specific.codecSpecific.VP9.gof.CopyGofInfoVP9(gof_);
      }
      webrtc::vp9::GetQp(p, size, &encoded_image_.qp_);
    } else if (codec_ == MFX_CODEC_AVC) {
      codec_specific.codecType = webrtc::kVideoCodecH264;
      codec_specific.codecSpecific.H264.packetization_mode =
          webrtc::H264PacketizationMode::NonInterleaved;

      h264_bitstream_parser_.ParseBitstream(encoded_image_);
      encoded_image_.qp_ = h264_bitstream_parser_.GetLastSliceQp().value_or(-1);
    } else if (codec_ == MFX_CODEC_HEVC) {
      codec_specific.codecType = webrtc::kVideoCodecH265;

      h265_bitstream_parser_.ParseBitstream(encoded_image_);
      encoded_image_.qp_ = h265_bitstream_parser_.GetLastSliceQp().value_or(-1);
    } else if (codec_ == MFX_CODEC_AV1) {
      codec_specific.codecType = webrtc::kVideoCodecAV1;

      bool is_key =
          encoded_image_._frameType == webrtc::VideoFrameType::kVideoFrameKey;
      std::vector<webrtc::ScalableVideoController::LayerFrameConfig>
          layer_frames = svc_controller_->NextFrameConfig(is_key);
      // AV1 の SVC では、まれにエンコード対象のレイヤーフレームが存在しない場合がある。
      // 次のフレームを待つことで正常に継続可能なケースであるため、エラーではなく正常終了で返してスキップする。
      if (layer_frames.empty()) {
        return WEBRTC_VIDEO_CODEC_OK;
      }
      codec_specific.end_of_picture = true;
      codec_specific.scalability_mode = scalability_mode_;
      codec_specific.generic_frame_info =
          svc_controller_->OnEncodeDone(layer_frames[0]);
      if (is_key && codec_specific.generic_frame_info) {
        codec_specific.template_structure =
            svc_controller_->DependencyStructure();
        auto& resolutions = codec_specific.template_structure->resolutions;
        resolutions = {webrtc::RenderResolution(encoded_image_._encodedWidth,
                                                encoded_image_._encodedHeight)};
      }
    }

    webrtc::EncodedImageCallback::Result result =
        callback_->OnEncodedImage(encoded_image_, &codec_specific);
    if (result.error != webrtc::EncodedImageCallback::Result::OK) {
      RTC_LOG(LS_ERROR) << __FUNCTION__
                        << " OnEncodedImage failed error:" << result.error;
      return WEBRTC_VIDEO_CODEC_ERROR;
    }
    bitrate_adjuster_.Update(size);
  }

  return WEBRTC_VIDEO_CODEC_OK;
}
void VplVideoEncoderImpl::SetRates(const RateControlParameters& parameters) {
  if (parameters.framerate_fps < 1.0) {
    RTC_LOG(LS_WARNING) << "Invalid frame rate: " << parameters.framerate_fps;
    return;
  }

  uint32_t new_framerate = (uint32_t)parameters.framerate_fps;
  uint32_t new_bitrate = parameters.bitrate.get_sum_bps();
  RTC_LOG(LS_INFO) << __FUNCTION__ << " framerate_:" << framerate_
                   << " new_framerate: " << new_framerate
                   << " target_bitrate_bps_:" << target_bitrate_bps_
                   << " new_bitrate:" << new_bitrate
                   << " max_bitrate_bps_:" << max_bitrate_bps_;
  framerate_ = new_framerate;
  target_bitrate_bps_ = new_bitrate;
  bitrate_adjuster_.SetTargetBitrateBps(target_bitrate_bps_);
  reconfigure_needed_ = true;

  // bitrate が 0 の時レイヤーを無効にする
  if (svc_controller_) {
    svc_controller_->OnRatesUpdated(parameters.bitrate);
  }
}
webrtc::VideoEncoder::EncoderInfo VplVideoEncoderImpl::GetEncoderInfo() const {
  webrtc::VideoEncoder::EncoderInfo info;
  info.supports_native_handle = true;
  info.implementation_name = "libvpl";
  info.scaling_settings = webrtc::VideoEncoder::ScalingSettings(
      kLowH264QpThreshold, kHighH264QpThreshold);
  info.is_hardware_accelerated = true;
  return info;
}

int32_t VplVideoEncoderImpl::InitVpl() {
  encoder_ = CreateEncoder(session_, codec_, width_, height_, framerate_,
                           bitrate_adjuster_.GetAdjustedBitrateBps() / 1000,
                           max_bitrate_bps_ / 1000, true);
  if (encoder_ == nullptr) {
    RTC_LOG(LS_ERROR) << "Failed to create encoder";
    return WEBRTC_VIDEO_CODEC_ERROR;
  }

  mfxStatus sts = MFX_ERR_NONE;

  mfxVideoParam param;
  memset(&param, 0, sizeof(param));

  // Retrieve video parameters selected by encoder.
  // - BufferSizeInKB parameter is required to set bit stream buffer size
  sts = encoder_->GetVideoParam(&param);
  VPL_CHECK_RESULT(sts, MFX_ERR_NONE, sts);
  RTC_LOG(LS_INFO) << "BufferSizeInKB=" << param.mfx.BufferSizeInKB;

  // Query number of required surfaces for encoder
  memset(&alloc_request_, 0, sizeof(alloc_request_));
  sts = encoder_->QueryIOSurf(&param, &alloc_request_);
  VPL_CHECK_RESULT(sts, MFX_ERR_NONE, sts);

  RTC_LOG(LS_INFO) << "Encoder NumFrameSuggested="
                   << alloc_request_.NumFrameSuggested;

  frame_info_ = param.mfx.FrameInfo;

  // 出力ビットストリームの初期化
  bitstream_buffer_.resize(param.mfx.BufferSizeInKB * 1000);

  memset(&bitstream_, 0, sizeof(bitstream_));
  bitstream_.MaxLength = bitstream_buffer_.size();
  bitstream_.Data = bitstream_buffer_.data();

#ifdef __linux__
  // DMABUF モードの初期化を試みる
  if (EnableDmaBufMode()) {
    RTC_LOG(LS_INFO) << "Using DMABUF mode for zero-copy";
    // DMABUF モードではフレームアロケータがサーフェースを管理
    return WEBRTC_VIDEO_CODEC_OK;
  }
  RTC_LOG(LS_INFO)
      << "DMABUF mode not available, falling back to system memory";
#endif

  // 必要な枚数分の入力サーフェスを作る（システムメモリ）
  {
    int width = (alloc_request_.Info.Width + 31) / 32 * 32;
    int height = (alloc_request_.Info.Height + 31) / 32 * 32;
    // 1枚あたりのバイト数
    // NV12 なので 1 ピクセルあたり 12 ビット
    int size = width * height * 12 / 8;
    surface_buffer_.resize(alloc_request_.NumFrameSuggested * size);

    surfaces_.clear();
    surfaces_.reserve(alloc_request_.NumFrameSuggested);
    for (int i = 0; i < alloc_request_.NumFrameSuggested; i++) {
      mfxFrameSurface1 surface;
      memset(&surface, 0, sizeof(surface));
      surface.Info = frame_info_;
      surface.Data.Y = surface_buffer_.data() + i * size;
      surface.Data.U = surface_buffer_.data() + i * size + width * height;
      surface.Data.V = surface_buffer_.data() + i * size + width * height + 1;
      surface.Data.Pitch = width;
      surfaces_.push_back(surface);
    }
  }

  return WEBRTC_VIDEO_CODEC_OK;
}
int32_t VplVideoEncoderImpl::ReleaseVpl() {
  if (encoder_ != nullptr) {
    encoder_->Close();
  }
  encoder_.reset();

#ifdef __linux__
  if (use_dmabuf_) {
    frame_allocator_.reset();
    if (va_display_) {
      vaTerminate(va_display_);
      va_display_ = nullptr;
    }
    dmabuf_fds_.clear();
    use_dmabuf_ = false;
  }
#endif

  return WEBRTC_VIDEO_CODEC_OK;
}

////////////////////////
// VplVideoEncoder
////////////////////////

bool VplVideoEncoder::IsSupported(std::shared_ptr<VplSession> session,
                                  webrtc::VideoCodecType codec) {
  if (session == nullptr) {
    return false;
  }

  auto encoder = VplVideoEncoderImpl::CreateEncoder(
      session, ToMfxCodec(codec), 1920, 1080, 30, 10, 20, false);
  bool result = encoder != nullptr;
  RTC_LOG(LS_VERBOSE) << "IsSupported: codec="
                      << CodecToString(ToMfxCodec(codec))
                      << " result=" << result;
  return result;
}

std::unique_ptr<VplVideoEncoder> VplVideoEncoder::Create(
    std::shared_ptr<VplSession> session,
    webrtc::VideoCodecType codec) {
  return std::unique_ptr<VplVideoEncoder>(
      new VplVideoEncoderImpl(session, ToMfxCodec(codec)));
}

bool VplVideoEncoderImpl::EnableDmaBufMode() {
#ifdef __linux__
  // VA-API ディスプレイをオープン
  int drm_fd = open("/dev/dri/renderD128", O_RDWR);
  if (drm_fd < 0) {
    // renderD129 を試す
    drm_fd = open("/dev/dri/renderD129", O_RDWR);
    if (drm_fd < 0) {
      RTC_LOG(LS_WARNING) << "Failed to open DRM device";
      return false;
    }
  }

  va_display_ = vaGetDisplayDRM(drm_fd);
  if (!va_display_) {
    RTC_LOG(LS_WARNING) << "Failed to get VA display";
    close(drm_fd);
    return false;
  }

  int major, minor;
  VAStatus va_status = vaInitialize(va_display_, &major, &minor);
  if (va_status != VA_STATUS_SUCCESS) {
    RTC_LOG(LS_WARNING) << "Failed to initialize VA display: " << va_status;
    close(drm_fd);
    return false;
  }

  // フレームアロケータを作成
  frame_allocator_ = std::make_unique<VplFrameAllocator>();
  mfxStatus sts = frame_allocator_->Init(va_display_);
  if (sts != MFX_ERR_NONE) {
    RTC_LOG(LS_WARNING) << "Failed to init frame allocator: " << sts;
    vaTerminate(va_display_);
    close(drm_fd);
    return false;
  }

  // VPP を初期化 (YUY2 -> NV12 変換)
  vpp_ = std::make_unique<VplVideoProcessor>(session_);
  if (!vpp_->Init(width_, height_, framerate_)) {
    RTC_LOG(LS_WARNING) << "Failed to init VPP";
    vaTerminate(va_display_);
    close(drm_fd);
    return false;
  }

  // VPP にフレームアロケータを設定
  if (!vpp_->SetFrameAllocator(frame_allocator_.get())) {
    RTC_LOG(LS_WARNING) << "Failed to set frame allocator to VPP";
    vaTerminate(va_display_);
    close(drm_fd);
    return false;
  }

  // エンコーダーにフレームアロケータを設定
  // 注: 現在の VPL では SetFrameAllocator は直接サポートされていない
  // GPU メモリはセッションを通じて管理される

  // エンコーダー用の NV12 サーフェースを作成
  mfxFrameAllocRequest encoder_request;
  memset(&encoder_request, 0, sizeof(encoder_request));
  encoder_request.Info = alloc_request_.Info;
  encoder_request.Info.FourCC = MFX_FOURCC_NV12;  // NV12 フォーマット
  encoder_request.NumFrameSuggested = alloc_request_.NumFrameSuggested;
  encoder_request.Type = MFX_MEMTYPE_VIDEO_MEMORY_ENCODER_TARGET;

  mfxFrameAllocResponse encoder_response;
  sts = frame_allocator_->Alloc(&encoder_request, &encoder_response);
  if (sts != MFX_ERR_NONE) {
    RTC_LOG(LS_WARNING) << "Failed to allocate encoder surfaces: " << sts;
    vaTerminate(va_display_);
    close(drm_fd);
    return false;
  }

  // エンコーダー用サーフェースを設定
  surfaces_.clear();
  surfaces_.reserve(encoder_response.NumFrameActual);
  for (int i = 0; i < encoder_response.NumFrameActual; i++) {
    mfxFrameSurface1 surface;
    memset(&surface, 0, sizeof(surface));
    surface.Info = encoder_request.Info;
    surface.Data.MemId = encoder_response.mids[i];
    surfaces_.push_back(surface);
  }

  // V4L2 用の DMABUF fd を取得 (VPP 入力サーフェースから)
  dmabuf_fds_.clear();
  for (auto& surface : vpp_->GetInputSurfaces()) {
    int fd = frame_allocator_->GetDmaBufFd(surface.Data.MemId);
    dmabuf_fds_.push_back(fd);
  }

  use_dmabuf_ = true;

  RTC_LOG(LS_INFO) << "DMABUF mode enabled with VPP: "
                   << vpp_->GetInputSurfaces().size()
                   << " input surfaces (YUY2), " << surfaces_.size()
                   << " encoder surfaces (NV12)";
  return true;
#else
  return false;
#endif
}

std::vector<int> VplVideoEncoderImpl::GetDmaBufFds() const {
#ifdef __linux__
  return dmabuf_fds_;
#else
  return {};
#endif
}

void VplVideoEncoderImpl::OnVppSurfaceReady(int surface_index) {
#ifdef __linux__
  if (!use_dmabuf_ || !vpp_) {
    return;
  }

  // VPP 入力サーフェースを取得
  auto& input_surfaces = vpp_->GetInputSurfaces();
  if (surface_index >= input_surfaces.size()) {
    RTC_LOG(LS_ERROR) << "Invalid surface index: " << surface_index;
    return;
  }

  mfxFrameSurface1* input_surface = &input_surfaces[surface_index];
  mfxFrameSurface1* output_surface = nullptr;

  // VPP で YUY2 -> NV12 変換
  mfxStatus sts = vpp_->ProcessFrame(input_surface, &output_surface);
  if (sts != MFX_ERR_NONE || !output_surface) {
    RTC_LOG(LS_ERROR) << "VPP ProcessFrame failed: " << sts;
    return;
  }

  // エンコーダーで NV12 -> AV1/H264/H265 エンコード
  // (ここではサーフェースをキューに追加するだけ)
  current_vpp_surface_index_ = surface_index;

  // TODO: エンコーダーの Encode メソッドを呼び出す
  // ここで output_surface を使ってエンコード処理を実行
#endif
}

}  // namespace sora
