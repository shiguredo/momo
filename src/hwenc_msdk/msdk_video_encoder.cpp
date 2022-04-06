#include "msdk_video_encoder.h"

// libyuv
#include <libyuv.h>

const int kLowH264QpThreshold = 34;
const int kHighH264QpThreshold = 40;

MsdkVideoEncoder::MsdkVideoEncoder(const cricket::VideoCodec& codec)
    : bitrate_adjuster_(0.5, 0.95) {
  mfxStatus sts = MFX_ERR_NONE;

  mfxIMPL impl = MFX_IMPL_HARDWARE;
  mfxVersion ver = {{0, 1}};

  // Initialize Intel Media SDK Session
  sts = session_.Init(impl, &ver);
  MSDK_CHECK_RESULT(sts, MFX_ERR_NONE, sts);

  // Create VA display
  libva_ = CreateDRMLibVA();
  if (!libva_) {
    RTC_LOG(LS_ERROR) << "Failed to create DRM VA display";
    throw 1;
  }

  // Provide VA display handle to Media SDK
  sts = session_.SetHandle(static_cast<mfxHandleType>(MFX_HANDLE_VA_DISPLAY),
                           libva_->GetVADisplay());
  MSDK_CHECK_RESULT(sts, MFX_ERR_NONE, sts);

  // Query selected implementation and version
  sts = session_.QueryIMPL(&impl);
  MSDK_CHECK_RESULT(sts, MFX_ERR_NONE, sts);

  sts = session_.QueryVersion(&ver);
  MSDK_CHECK_RESULT(sts, MFX_ERR_NONE, sts);

  RTC_LOG(LS_INFO) << "Intel Media SDK Implementation: "
                   << (impl == MFX_IMPL_SOFTWARE ? "SOFTWARE" : "HARDWARE");
  RTC_LOG(LS_INFO) << "Intel Media SDK API Version: " << ver.Major << "."
                   << ver.Minor;
}
MsdkVideoEncoder::~MsdkVideoEncoder() {}

bool MsdkVideoEncoder::IsSupported(mfxU32 codec) {
  std::unique_ptr<DRMLibVA> libva = CreateDRMLibVA();
  if (!libva) {
    return false;
  }

  MFXVideoSession session;

  mfxStatus sts = MFX_ERR_NONE;

  sts = session.SetHandle(static_cast<mfxHandleType>(MFX_HANDLE_VA_DISPLAY),
                          libva->GetVADisplay());
  if (sts != MFX_ERR_NONE) {
    return false;
  }

  mfxIMPL impl = MFX_IMPL_HARDWARE;
  sts = session.QueryIMPL(&impl);
  if (sts != MFX_ERR_NONE) {
    return false;
  }

  mfxVersion ver = {{0, 1}};
  sts = session.QueryVersion(&ver);
  if (sts != MFX_ERR_NONE) {
    return false;
  }

  std::unique_ptr<MFXVideoENCODE> encoder;
  encoder.reset(new MFXVideoENCODE(session));

  int framerate = 30;
  int width = 640;
  int height = 480;

  mfxVideoParam param;
  memset(&param, 0, sizeof(param));

  param.mfx.CodecId = codec;
  if (codec == MFX_CODEC_VP8) {
    param.mfx.CodecProfile = MFX_PROFILE_VP8_0;
  } else if (codec == MFX_CODEC_VP9) {
    param.mfx.CodecProfile = MFX_PROFILE_VP9_0;
  } else if (codec == MFX_CODEC_AVC) {
    //param.mfx.CodecProfile = MFX_PROFILE_AVC_HIGH;
    //param.mfx.CodecLevel = MFX_LEVEL_AVC_51;
    param.mfx.CodecProfile = MFX_PROFILE_AVC_BASELINE;
    param.mfx.CodecLevel = MFX_LEVEL_AVC_1;
  } else if (codec == MFX_CODEC_AV1) {
    param.mfx.CodecProfile = MFX_PROFILE_AV1_MAIN;
  }
  param.mfx.TargetUsage = MFX_TARGETUSAGE_BALANCED;
  param.mfx.TargetKbps = 3000;
  param.mfx.MaxKbps = 3000;
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
  int ext_buffers_size = 0;
  if (codec == MFX_CODEC_AVC) {
    mfxExtCodingOption ext_coding_option;
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

    mfxExtCodingOption2 ext_coding_option2;
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

  // MFX_ERR_NONE	The function completed successfully.
  // MFX_ERR_UNSUPPORTED	The function failed to identify a specific implementation for the required features.
  // MFX_WRN_PARTIAL_ACCELERATION	The underlying hardware does not fully support the specified video parameters; The encoding may be partially accelerated. Only SDK HW implementations may return this status code.
  // MFX_WRN_INCOMPATIBLE_VIDEO_PARAM	The function detected some video parameters were incompatible with others; incompatibility resolved.
  sts = encoder->Query(&param, &param);
  if (sts == MFX_ERR_NONE || sts == MFX_ERR_UNSUPPORTED) {
    return false;
  }

  return true;
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
    param.mfx.TargetKbps = bitrate_adjuster_.GetAdjustedBitrateBps() / 1000;
    param.mfx.FrameInfo.FrameRateExtN = framerate_;
    param.mfx.FrameInfo.FrameRateExtD = 1;

    sts = encoder_->Reset(&param);
    MSDK_CHECK_RESULT(sts, MFX_ERR_NONE, sts);

    reconfigure_needed_ = false;
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

  sts = session_.SyncOperation(syncp, 600000);
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
    encoded_image_.set_size(size);
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
    encoded_image_._frameType = webrtc::VideoFrameType::kVideoFrameDelta;

    webrtc::CodecSpecificInfo codec_specific;
    codec_specific.codecType = webrtc::kVideoCodecH264;
    codec_specific.codecSpecific.H264.packetization_mode =
        webrtc::H264PacketizationMode::NonInterleaved;

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
  encoder_.reset(new MFXVideoENCODE(session_));

  // Set required video parameters for encode
  // - In this example we are encoding an AVC (H.264) stream
  mfxVideoParam param;
  memset(&param, 0, sizeof(param));

  param.mfx.CodecId = MFX_CODEC_AVC;
  //param.mfx.CodecProfile = MFX_PROFILE_AVC_HIGH;
  //param.mfx.CodecLevel = MFX_LEVEL_AVC_51;
  param.mfx.CodecProfile = MFX_PROFILE_AVC_BASELINE;
  param.mfx.CodecLevel = MFX_LEVEL_AVC_1;
  param.mfx.TargetUsage = MFX_TARGETUSAGE_BALANCED;
  param.mfx.TargetKbps = bitrate_adjuster_.GetAdjustedBitrateBps() / 1000;
  param.mfx.MaxKbps = max_bitrate_bps_ / 1000;
  param.mfx.RateControlMethod = MFX_RATECONTROL_VBR;
  //param.mfx.NumSlice = 1;
  //param.mfx.NumRefFrame = 1;
  param.mfx.FrameInfo.FrameRateExtN = framerate_;
  param.mfx.FrameInfo.FrameRateExtD = 1;
  param.mfx.FrameInfo.FourCC = MFX_FOURCC_NV12;
  param.mfx.FrameInfo.ChromaFormat = MFX_CHROMAFORMAT_YUV420;
  param.mfx.FrameInfo.PicStruct = MFX_PICSTRUCT_PROGRESSIVE;
  param.mfx.FrameInfo.CropX = 0;
  param.mfx.FrameInfo.CropY = 0;
  param.mfx.FrameInfo.CropW = width_;
  param.mfx.FrameInfo.CropH = height_;
  // Width must be a multiple of 16
  // Height must be a multiple of 16 in case of frame picture and a multiple of 32 in case of field picture
  param.mfx.FrameInfo.Width = (width_ + 15) / 16 * 16;
  param.mfx.FrameInfo.Height = (height_ + 15) / 16 * 16;

  //param.mfx.GopOptFlag = MFX_GOP_STRICT | MFX_GOP_CLOSED;
  //param.mfx.IdrInterval = codec_settings->H264().keyFrameInterval;
  //param.mfx.IdrInterval = 0;
  param.mfx.GopRefDist = 1;
  //param.mfx.EncodedOrder = 0;
  param.AsyncDepth = 1;
  param.IOPattern =
      MFX_IOPATTERN_IN_SYSTEM_MEMORY | MFX_IOPATTERN_OUT_SYSTEM_MEMORY;

  mfxExtCodingOption ext_coding_option;
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

  mfxExtCodingOption2 ext_coding_option2;
  memset(&ext_coding_option2, 0, sizeof(ext_coding_option2));
  ext_coding_option2.Header.BufferId = MFX_EXTBUFF_CODING_OPTION2;
  ext_coding_option2.Header.BufferSz = sizeof(ext_coding_option2);
  ext_coding_option2.RepeatPPS = MFX_CODINGOPTION_ON;
  //ext_coding_option2.MaxSliceSize = 1;
  //ext_coding_option2.AdaptiveI = MFX_CODINGOPTION_ON;

  mfxExtBuffer* ext_buffers[2];
  ext_buffers[0] = (mfxExtBuffer*)&ext_coding_option;
  ext_buffers[1] = (mfxExtBuffer*)&ext_coding_option2;
  param.ExtParam = ext_buffers;
  param.NumExtParam = sizeof(ext_buffers) / sizeof(ext_buffers[0]);

  mfxStatus sts = MFX_ERR_NONE;

  // Validate video encode parameters (optional)
  // - In this example the validation result is written to same structure
  // - MFX_WRN_INCOMPATIBLE_VIDEO_PARAM is returned if some of the video parameters are not supported,
  //   instead the encoder will select suitable parameters closest matching the requested configuration
  sts = encoder_->Query(&param, &param);
  //MSDK_IGNORE_MFX_STS(sts, MFX_WRN_INCOMPATIBLE_VIDEO_PARAM);
  MSDK_CHECK_RESULT(sts, MFX_ERR_NONE, sts);

  // Query number of required surfaces for encoder
  memset(&alloc_request_, 0, sizeof(alloc_request_));
  sts = encoder_->QueryIOSurf(&param, &alloc_request_);
  MSDK_CHECK_RESULT(sts, MFX_ERR_NONE, sts);

  RTC_LOG(LS_INFO) << "Encoder NumFrameSuggested="
                   << alloc_request_.NumFrameSuggested;

  // Initialize the Media SDK encoder
  sts = encoder_->Init(&param);
  //MSDK_IGNORE_MFX_STS(sts, MFX_WRN_PARTIAL_ACCELERATION);
  MSDK_CHECK_RESULT(sts, MFX_ERR_NONE, sts);

  // Retrieve video parameters selected by encoder.
  // - BufferSizeInKB parameter is required to set bit stream buffer size
  memset(&param, 0, sizeof(param));
  sts = encoder_->GetVideoParam(&param);
  MSDK_CHECK_RESULT(sts, MFX_ERR_NONE, sts);

  RTC_LOG(LS_INFO) << "BufferSizeInKB=" << param.mfx.BufferSizeInKB;

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
  return WEBRTC_VIDEO_CODEC_OK;
}