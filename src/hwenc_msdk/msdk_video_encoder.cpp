#include "msdk_video_encoder.h"

#include <iostream>

// libyuv
#include <libyuv.h>

#include "msdk_utils.h"

const int kLowH264QpThreshold = 34;
const int kHighH264QpThreshold = 40;

MsdkVideoEncoder::MsdkVideoEncoder(std::shared_ptr<MsdkSession> session,
                                   mfxU32 codec)
    : session_(session), codec_(codec), bitrate_adjuster_(0.5, 0.95) {}
MsdkVideoEncoder::~MsdkVideoEncoder() {}

std::unique_ptr<MFXVideoENCODE> MsdkVideoEncoder::CreateEncoder(
    std::shared_ptr<MsdkSession> session,
    mfxU32 codec,
    int width,
    int height,
    int framerate,
    int target_kbps,
    int max_kbps,
    bool init) {
  mfxStatus sts = MFX_ERR_NONE;

  mfxVideoParam param;
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
  } else if (codec == MFX_CODEC_AV1) {
    //param.mfx.CodecProfile = MFX_PROFILE_AV1_MAIN;
  }
  param.mfx.TargetUsage = MFX_TARGETUSAGE_BALANCED;
  //param.mfx.BRCParamMultiplier = 1;
  //param.mfx.InitialDelayInKB = target_kbps;
  param.mfx.TargetKbps = target_kbps;
  param.mfx.MaxKbps = max_kbps;
  param.mfx.RateControlMethod = MFX_RATECONTROL_VBR;
  //param.mfx.NumSlice = 1;
  //param.mfx.NumRefFrame = 1;
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

  //param.mfx.GopOptFlag = MFX_GOP_STRICT | MFX_GOP_CLOSED;
  //param.mfx.IdrInterval = codec_settings->H264().keyFrameInterval;
  //param.mfx.IdrInterval = 0;
  param.mfx.GopRefDist = 1;
  //param.mfx.EncodedOrder = 0;
  param.AsyncDepth = 1;
  param.IOPattern =
      MFX_IOPATTERN_IN_SYSTEM_MEMORY | MFX_IOPATTERN_OUT_SYSTEM_MEMORY;

  mfxExtBuffer* ext_buffers[10];
  mfxExtCodingOption ext_coding_option;
  mfxExtCodingOption2 ext_coding_option2;
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
  }

  if (ext_buffers_size != 0) {
    param.ExtParam = ext_buffers;
    param.NumExtParam = ext_buffers_size;
  }

  std::unique_ptr<MFXVideoENCODE> encoder(new MFXVideoENCODE(session->session));

  // MFX_ERR_NONE	The function completed successfully.
  // MFX_ERR_UNSUPPORTED	The function failed to identify a specific implementation for the required features.
  // MFX_WRN_PARTIAL_ACCELERATION	The underlying hardware does not fully support the specified video parameters; The encoding may be partially accelerated. Only SDK HW implementations may return this status code.
  // MFX_WRN_INCOMPATIBLE_VIDEO_PARAM	The function detected some video parameters were incompatible with others; incompatibility resolved.
  mfxVideoParam bk_param;
  memcpy(&bk_param, &param, sizeof(bk_param));
  sts = encoder->Query(&param, &param);
  if (sts == MFX_ERR_UNSUPPORTED) {
    memcpy(&param, &bk_param, sizeof(bk_param));

    // 失敗したら LowPower ON にした状態でもう一度確認する
    param.mfx.LowPower = MFX_CODINGOPTION_ON;
    if (codec == MFX_CODEC_AVC) {
      param.mfx.RateControlMethod = MFX_RATECONTROL_CQP;
      param.mfx.QPI = 25;
      param.mfx.QPP = 33;
      param.mfx.QPB = 40;
      //param.IOPattern = MFX_IOPATTERN_IN_SYSTEM_MEMORY;
    }
    memcpy(&bk_param, &param, sizeof(bk_param));
    sts = encoder->Query(&param, &param);
    if (sts < 0) {
      const char* codec_str = codec == MFX_CODEC_VP8   ? "MFX_CODEC_VP8"
                              : codec == MFX_CODEC_VP9 ? "MFX_CODEC_VP9"
                              : codec == MFX_CODEC_AV1 ? "MFX_CODEC_AV1"
                              : codec == MFX_CODEC_AVC ? "MFX_CODEC_AVC"
                                                       : "MFX_CODEC_UNKNOWN";
      //std::cerr << "Unsupported encoder codec: codec=" << codec_str
      //          << std::endl;
      return nullptr;
    }
  }

  //#define F(NAME)                                              \
  //  if (bk_param.NAME != param.NAME)                           \
  //  std::cout << "param " << #NAME << " old=" << bk_param.NAME \
  //            << " new=" << param.NAME << std::endl
  //
  //  F(mfx.LowPower);
  //  F(mfx.BRCParamMultiplier);
  //  F(mfx.FrameInfo.FrameRateExtN);
  //  F(mfx.FrameInfo.FrameRateExtD);
  //  F(mfx.FrameInfo.FourCC);
  //  F(mfx.FrameInfo.ChromaFormat);
  //  F(mfx.FrameInfo.PicStruct);
  //  F(mfx.FrameInfo.CropX);
  //  F(mfx.FrameInfo.CropY);
  //  F(mfx.FrameInfo.CropW);
  //  F(mfx.FrameInfo.CropH);
  //  F(mfx.FrameInfo.Width);
  //  F(mfx.FrameInfo.Height);
  //  F(mfx.CodecId);
  //  F(mfx.CodecProfile);
  //  F(mfx.CodecLevel);
  //  F(mfx.GopPicSize);
  //  F(mfx.GopRefDist);
  //  F(mfx.GopOptFlag);
  //  F(mfx.IdrInterval);
  //  F(mfx.TargetUsage);
  //  F(mfx.RateControlMethod);
  //  F(mfx.InitialDelayInKB);
  //  F(mfx.TargetKbps);
  //  F(mfx.MaxKbps);
  //  F(mfx.BufferSizeInKB);
  //  F(mfx.NumSlice);
  //  F(mfx.NumRefFrame);
  //  F(mfx.EncodedOrder);
  //  F(mfx.DecodedOrder);
  //  F(mfx.ExtendedPicStruct);
  //  F(mfx.TimeStampCalc);
  //  F(mfx.SliceGroupsPresent);
  //  F(mfx.MaxDecFrameBuffering);
  //  F(mfx.EnableReallocRequest);
  //  F(AsyncDepth);
  //  F(IOPattern);
  //#undef F

  //if (sts != MFX_ERR_NONE) {
  //  const char* codec_str = codec == MFX_CODEC_VP8   ? "MFX_CODEC_VP8"
  //                          : codec == MFX_CODEC_VP9 ? "MFX_CODEC_VP9"
  //                          : codec == MFX_CODEC_AV1 ? "MFX_CODEC_AV1"
  //                          : codec == MFX_CODEC_AVC ? "MFX_CODEC_AVC"
  //                                                   : "MFX_CODEC_UNKNOWN";
  //  std::cerr << "Supported specified codec but has warning: codec="
  //            << codec_str << " sts=" << sts << std::endl;
  //}

  if (init) {
    sts = encoder->Init(&param);
    if (sts != MFX_ERR_NONE) {
      RTC_LOG(LS_ERROR) << "Failed to Init: sts=" << sts;
      return nullptr;
    }
  }

  return encoder;
}

bool MsdkVideoEncoder::IsSupported(std::shared_ptr<MsdkSession> session,
                                   mfxU32 codec) {
  auto encoder = CreateEncoder(session, codec, 1920, 1080, 30, 10, 20, false);
  return encoder != nullptr;
}

int32_t MsdkVideoEncoder::InitEncode(const webrtc::VideoCodec* codec_settings,
                                     int32_t number_of_cores,
                                     size_t max_payload_size) {
  RTC_DCHECK(codec_settings);
  RTC_DCHECK_EQ(codec_settings->codecType, webrtc::kVideoCodecH264);

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

  return InitMediaSDK();
}
int32_t MsdkVideoEncoder::RegisterEncodeCompleteCallback(
    webrtc::EncodedImageCallback* callback) {
  std::lock_guard<std::mutex> lock(mutex_);
  callback_ = callback;
  return WEBRTC_VIDEO_CODEC_OK;
}
int32_t MsdkVideoEncoder::Release() {
  return ReleaseMediaSDK();
}
int32_t MsdkVideoEncoder::Encode(
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

  // I420 から NV12 に変換
  rtc::scoped_refptr<const webrtc::I420BufferInterface> frame_buffer =
      frame.video_frame_buffer()->ToI420();
  libyuv::I420ToNV12(
      frame_buffer->DataY(), frame_buffer->StrideY(), frame_buffer->DataU(),
      frame_buffer->StrideU(), frame_buffer->DataV(), frame_buffer->StrideV(),
      surface->Data.Y, surface->Data.Pitch, surface->Data.U,
      surface->Data.Pitch, frame_buffer->width(), frame_buffer->height());

  mfxStatus sts;

  mfxEncodeCtrl ctrl;
  memset(&ctrl, 0, sizeof(ctrl));
  //send_key_frame = true;
  if (send_key_frame) {
    ctrl.FrameType = MFX_FRAMETYPE_I | MFX_FRAMETYPE_IDR | MFX_FRAMETYPE_REF;
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
    MSDK_CHECK_RESULT(sts, MFX_ERR_NONE, sts);

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
    MSDK_CHECK_RESULT(sts, MFX_ERR_NONE, sts);

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
  MSDK_CHECK_RESULT(sts, MFX_ERR_NONE, sts);

  sts = session_->session.SyncOperation(syncp, 600000);
  MSDK_CHECK_RESULT(sts, MFX_ERR_NONE, sts);

  //RTC_LOG(LS_ERROR) << "SurfaceSize=" << (surface->Data.U - surface->Data.Y);
  //RTC_LOG(LS_ERROR) << "DataLength=" << bitstream_.DataLength;
  {
    uint8_t* p = bitstream_.Data + bitstream_.DataOffset;
    int size = bitstream_.DataLength;
    bitstream_.DataLength = 0;

    //FILE* fp = fopen("test.mp4", "a+");
    //fwrite(p, 1, size, fp);
    //fclose(fp);

    auto buf = webrtc::EncodedImageBuffer::Create(p, size);
    encoded_image_.SetEncodedData(buf);
    encoded_image_._encodedWidth = width_;
    encoded_image_._encodedHeight = height_;
    encoded_image_.content_type_ =
        (mode_ == webrtc::VideoCodecMode::kScreensharing)
            ? webrtc::VideoContentType::SCREENSHARE
            : webrtc::VideoContentType::UNSPECIFIED;
    encoded_image_.timing_.flags = webrtc::VideoSendTiming::kInvalid;
    encoded_image_.SetTimestamp(frame.timestamp());
    encoded_image_.ntp_time_ms_ = frame.ntp_time_ms();
    encoded_image_.capture_time_ms_ = frame.render_time_ms();
    encoded_image_.rotation_ = frame.rotation();
    encoded_image_.SetColorSpace(frame.color_space());
    if (bitstream_.FrameType == MFX_FRAMETYPE_I ||
        bitstream_.FrameType == MFX_FRAMETYPE_IDR) {
      encoded_image_._frameType = webrtc::VideoFrameType::kVideoFrameKey;
    } else {
      encoded_image_._frameType = webrtc::VideoFrameType::kVideoFrameDelta;
    }

    webrtc::CodecSpecificInfo codec_specific;
    if (codec_ == MFX_CODEC_AVC) {
      codec_specific.codecType = webrtc::kVideoCodecH264;
      codec_specific.codecSpecific.H264.packetization_mode =
          webrtc::H264PacketizationMode::NonInterleaved;
    }

    h264_bitstream_parser_.ParseBitstream(encoded_image_);
    encoded_image_.qp_ = h264_bitstream_parser_.GetLastSliceQp().value_or(-1);

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
void MsdkVideoEncoder::SetRates(const RateControlParameters& parameters) {
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
}
webrtc::VideoEncoder::EncoderInfo MsdkVideoEncoder::GetEncoderInfo() const {
  webrtc::VideoEncoder::EncoderInfo info;
  info.supports_native_handle = true;
  info.implementation_name = "NvCodec H264";
  info.scaling_settings = webrtc::VideoEncoder::ScalingSettings(
      kLowH264QpThreshold, kHighH264QpThreshold);
  info.is_hardware_accelerated = true;
  return info;
}

int32_t MsdkVideoEncoder::InitMediaSDK() {
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
  MSDK_CHECK_RESULT(sts, MFX_ERR_NONE, sts);
  RTC_LOG(LS_INFO) << "BufferSizeInKB=" << param.mfx.BufferSizeInKB;

  // Query number of required surfaces for encoder
  memset(&alloc_request_, 0, sizeof(alloc_request_));
  sts = encoder_->QueryIOSurf(&param, &alloc_request_);
  MSDK_CHECK_RESULT(sts, MFX_ERR_NONE, sts);

  RTC_LOG(LS_INFO) << "Encoder NumFrameSuggested="
                   << alloc_request_.NumFrameSuggested;

  frame_info_ = param.mfx.FrameInfo;

  // 出力ビットストリームの初期化
  bitstream_buffer_.resize(param.mfx.BufferSizeInKB * 1000);

  memset(&bitstream_, 0, sizeof(bitstream_));
  bitstream_.MaxLength = bitstream_buffer_.size();
  bitstream_.Data = bitstream_buffer_.data();

  // 必要な枚数分の入力サーフェスを作る
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
int32_t MsdkVideoEncoder::ReleaseMediaSDK() {
  if (encoder_ != nullptr) {
    encoder_->Close();
  }
  encoder_.reset();
  return WEBRTC_VIDEO_CODEC_OK;
}