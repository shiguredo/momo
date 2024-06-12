/*
 *  Copyright (c) 2015 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 *
 */

#include "jetson_video_encoder.h"

#include <limits>
#include <string>

// WebRTC
#include <common_video/libyuv/include/webrtc_libyuv.h>
#include <modules/video_coding/include/video_codec_interface.h>
#include <modules/video_coding/include/video_error_codes.h>
#include <modules/video_coding/svc/create_scalability_structure.h>
#include <rtc_base/checks.h>
#include <rtc_base/logging.h>
#include <rtc_base/time_utils.h>
#include <system_wrappers/include/metrics.h>
#include <third_party/libyuv/include/libyuv/convert.h>
#include <third_party/libyuv/include/libyuv/convert_from.h>
#include <third_party/libyuv/include/libyuv/video_common.h>

// L4T Multimedia API
#include <NvBufSurface.h>
#include <NvVideoEncoder.h>
#include <nvbufsurface.h>

#include "jetson_buffer.h"

#define H264HWENC_HEADER_DEBUG 0
#define INIT_ERROR(cond, desc)                 \
  if (cond) {                                  \
    RTC_LOG(LS_ERROR) << __FUNCTION__ << desc; \
    Release();                                 \
    return WEBRTC_VIDEO_CODEC_ERROR;           \
  }

JetsonVideoEncoder::JetsonVideoEncoder(const cricket::VideoCodec& codec)
    : callback_(nullptr),
      encoder_(nullptr),
      configured_framerate_(30),
      use_native_(false),
      use_dmabuff_(false) {}

JetsonVideoEncoder::~JetsonVideoEncoder() {
  Release();
}

// 標準出力や標準エラーに出力されないようにいろいろする
//struct SuppressErrors {
//  SuppressErrors() {
//    old_stdout = stdout;
//    old_stderr = stderr;
//    old_log_level = log_level;
//    stdout = fopen("/dev/null", "w");
//    stderr = fopen("/dev/null", "w");
//    log_level = -1;
//  }
//  ~SuppressErrors() {
//    fclose(stdout);
//    fclose(stderr);
//    stdout = old_stdout;
//    stderr = old_stderr;
//    log_level = old_log_level;
//  }
//  FILE* old_stdout;
//  FILE* old_stderr;
//  int old_log_level;
//};

bool JetsonVideoEncoder::IsSupportedVP8() {
  //SuppressErrors sup;

  auto encoder = NvVideoEncoder::createVideoEncoder("enc0");
  auto ret = encoder->setCapturePlaneFormat(V4L2_PIX_FMT_VP8, 1024, 768,
                                            2 * 1024 * 1024);
  delete encoder;

  return ret >= 0;
}

bool JetsonVideoEncoder::IsSupportedVP9() {
  //SuppressErrors sup;

  auto encoder = NvVideoEncoder::createVideoEncoder("enc0");
  auto ret = encoder->setCapturePlaneFormat(V4L2_PIX_FMT_VP9, 1024, 768,
                                            2 * 1024 * 1024);
  delete encoder;

  return ret >= 0;
}

bool JetsonVideoEncoder::IsSupportedAV1() {
  //SuppressErrors sup;

  auto encoder = NvVideoEncoder::createVideoEncoder("enc0");
  auto ret = encoder->setCapturePlaneFormat(V4L2_PIX_FMT_AV1, 1024, 768,
                                            2 * 1024 * 1024);
  delete encoder;

  return ret >= 0;
}

int32_t JetsonVideoEncoder::InitEncode(const webrtc::VideoCodec* codec_settings,
                                       int32_t number_of_cores,
                                       size_t max_payload_size) {
  RTC_DCHECK(codec_settings);

  int32_t release_ret = Release();
  if (release_ret != WEBRTC_VIDEO_CODEC_OK) {
    return release_ret;
  }
  if (&codec_ != codec_settings) {
    codec_ = *codec_settings;
  }

  width_ = codec_settings->width;
  height_ = codec_settings->height;
  target_bitrate_bps_ = codec_settings->startBitrate * 1000;
  if (codec_settings->codecType == webrtc::kVideoCodecH264) {
    key_frame_interval_ = codec_settings->H264().keyFrameInterval;
  } else if (codec_settings->codecType == webrtc::kVideoCodecVP8) {
    key_frame_interval_ = codec_settings->VP8().keyFrameInterval;
  } else if (codec_settings->codecType == webrtc::kVideoCodecVP9) {
    key_frame_interval_ = codec_settings->VP9().keyFrameInterval;
    RTC_LOG(LS_INFO) << "numberOfTemporalLayers: "
                     << codec_settings->VP9().numberOfTemporalLayers;
    RTC_LOG(LS_INFO) << "denoisingOn: " << codec_settings->VP9().denoisingOn;
    RTC_LOG(LS_INFO) << "keyFrameInterval: "
                     << codec_settings->VP9().keyFrameInterval;
    RTC_LOG(LS_INFO) << "adaptiveQpMode: "
                     << codec_settings->VP9().adaptiveQpMode;
    RTC_LOG(LS_INFO) << "automaticResizeOn: "
                     << codec_settings->VP9().automaticResizeOn;
    RTC_LOG(LS_INFO) << "numberOfSpatialLayers: "
                     << codec_settings->VP9().numberOfSpatialLayers;
    RTC_LOG(LS_INFO) << "interLayerPred: "
                     << codec_settings->VP9().interLayerPred;
  } else if (codec_settings->codecType == webrtc::kVideoCodecAV1) {
    auto scalability_mode = codec_settings->GetScalabilityMode();
    if (!scalability_mode) {
      RTC_LOG(LS_WARNING) << "Scalability mode is not set, using 'L1T1'.";
      scalability_mode = webrtc::ScalabilityMode::kL1T1;
    }
    RTC_LOG(LS_INFO) << "InitEncode scalability_mode:"
                     << (int)*scalability_mode;
    svc_controller_ = webrtc::CreateScalabilityStructure(*scalability_mode);
  }
  framerate_ = codec_settings->maxFramerate;

  RTC_LOG(LS_INFO) << "InitEncode " << framerate_ << "fps "
                   << target_bitrate_bps_ << "bit/sec　"
                   << codec_settings->maxBitrate << "kbit/sec　";

  // Initialize encoded image.
  encoded_image_.timing_.flags =
      webrtc::VideoSendTiming::TimingFrameFlags::kInvalid;
  encoded_image_.content_type_ =
      (codec_settings->mode == webrtc::VideoCodecMode::kScreensharing)
          ? webrtc::VideoContentType::SCREENSHARE
          : webrtc::VideoContentType::UNSPECIFIED;

  gof_.SetGofInfoVP9(webrtc::TemporalStructureMode::kTemporalStructureMode1);
  gof_idx_ = 0;
  RTC_LOG(LS_INFO) << __FUNCTION__ << " End";
  return WEBRTC_VIDEO_CODEC_OK;
}

int32_t JetsonVideoEncoder::Release() {
  JetsonRelease();
  return WEBRTC_VIDEO_CODEC_OK;
}

int32_t JetsonVideoEncoder::JetsonConfigure() {
  int ret = 0;
  bool use_converter =
      use_native_ && (width_ != raw_width_ || height_ != raw_height_ ||
                      decode_pixfmt_ != V4L2_PIX_FMT_YUV420M);

  encoder_ = NvVideoEncoder::createVideoEncoder("enc0");
  INIT_ERROR(!encoder_, "Failed to createVideoEncoder");

  if (codec_.codecType == webrtc::kVideoCodecH264) {
    ret = encoder_->setCapturePlaneFormat(V4L2_PIX_FMT_H264, width_, height_,
                                          2 * 1024 * 1024);
  } else if (codec_.codecType == webrtc::kVideoCodecVP8) {
    ret = encoder_->setCapturePlaneFormat(V4L2_PIX_FMT_VP8, width_, height_,
                                          2 * 1024 * 1024);
  } else if (codec_.codecType == webrtc::kVideoCodecVP9) {
    ret = encoder_->setCapturePlaneFormat(V4L2_PIX_FMT_VP9, width_, height_,
                                          2 * 1024 * 1024);
  } else if (codec_.codecType == webrtc::kVideoCodecAV1) {
    ret = encoder_->setCapturePlaneFormat(V4L2_PIX_FMT_AV1, width_, height_,
                                          2 * 1024 * 1024);
  }
  INIT_ERROR(ret < 0, "Failed to encoder setCapturePlaneFormat");

  ret = encoder_->setOutputPlaneFormat(V4L2_PIX_FMT_YUV420M, width_, height_);
  INIT_ERROR(ret < 0, "Failed to encoder setOutputPlaneFormat");

  if (codec_.codecType == webrtc::kVideoCodecH264) {
    ret = encoder_->setProfile(V4L2_MPEG_VIDEO_H264_PROFILE_HIGH);
    INIT_ERROR(ret < 0, "Failed to setProfile");

    ret = encoder_->setLevel(V4L2_MPEG_VIDEO_H264_LEVEL_5_1);
    INIT_ERROR(ret < 0, "Failed to setLevel");

    // 必須 なければ H264 でフレームレートが出ない
    ret = encoder_->setNumBFrames(0);
    INIT_ERROR(ret < 0, "Failed to setNumBFrames");

    ret = encoder_->setInsertSpsPpsAtIdrEnabled(true);
    INIT_ERROR(ret < 0, "Failed to setInsertSpsPpsAtIdrEnabled");

    ret = encoder_->setInsertVuiEnabled(true);
    INIT_ERROR(ret < 0, "Failed to setInsertSpsPpsAtIdrEnabled");

    // V4L2_ENC_HW_PRESET_ULTRAFAST が推奨値だけど MEDIUM でも Nano, AGX では OK
    // NX は V4L2_ENC_HW_PRESET_FAST でないとフレームレートがでない
    ret = encoder_->setHWPresetType(V4L2_ENC_HW_PRESET_FAST);
    INIT_ERROR(ret < 0, "Failed to setHWPresetType");
  } else if (codec_.codecType == webrtc::kVideoCodecVP8) {
    uint32_t qp_min =
        codec_.mode == webrtc::VideoCodecMode::kScreensharing ? 12 : 2;
    uint32_t qp_max = 56;
    if (codec_.qpMax >= qp_min) {
      qp_max = codec_.qpMax;
    }
    ret = encoder_->setQpRange(qp_min, qp_max, qp_min, qp_max, qp_min, qp_max);

    // V4L2_ENC_HW_PRESET_ULTRAFAST を指定しないと 30fps 出ない
    ret = encoder_->setHWPresetType(V4L2_ENC_HW_PRESET_ULTRAFAST);
    INIT_ERROR(ret < 0, "Failed to setHWPresetType");
  } else if (codec_.codecType == webrtc::kVideoCodecVP9) {
    // QP:150 が 30fps が出る下限。これ以上下げると 30fps を割る
    ret = encoder_->setQpRange(QP_RETAIN_VAL, 150, QP_RETAIN_VAL, 150,
                               QP_RETAIN_VAL, 150);
    INIT_ERROR(ret < 0, "Failed to setQpRange");

    // V4L2_ENC_HW_PRESET_ULTRAFAST が推奨値だけど SLOW でもフレームレートの落ち方が変わらない
    ret = encoder_->setHWPresetType(V4L2_ENC_HW_PRESET_SLOW);
    INIT_ERROR(ret < 0, "Failed to setHWPresetType");
  }

  ret = encoder_->setRateControlMode(V4L2_MPEG_VIDEO_BITRATE_MODE_CBR);
  INIT_ERROR(ret < 0, "Failed to setRateControlMode");

  /* ここに来たということはエンコーダは初期化されている
     初期化されているということは設定するべきは調整されたレートではなく
     最初の目標値であるべき BitrateAdjuster も初期化する*/
  bitrate_adjuster_.reset(new webrtc::BitrateAdjuster(.5, .95));
  bitrate_adjuster_->SetTargetBitrateBps(target_bitrate_bps_);
  SetBitrateBps(target_bitrate_bps_);

  ret = encoder_->setIDRInterval(key_frame_interval_);
  INIT_ERROR(ret < 0, "Failed to setIDRInterval");

  ret = encoder_->setIFrameInterval(0);
  INIT_ERROR(ret < 0, "Failed to setIFrameInterval");

  ret = encoder_->setFrameRate(framerate_, 1);
  INIT_ERROR(ret < 0, "Failed to setFrameRate");

  if (use_native_) {
    if (use_dmabuff_ || use_converter) {
      ret = encoder_->output_plane.reqbufs(V4L2_MEMORY_DMABUF, 10);
      INIT_ERROR(ret < 0, "Failed to reqbufs at encoder output_plane");

      int fd;
      NvBufSurf::NvCommonAllocateParams cParams;
      cParams.width = width_;
      cParams.height = height_;
      cParams.layout = NVBUF_LAYOUT_PITCH;
      cParams.colorFormat = NVBUF_COLOR_FORMAT_YUV420;
      cParams.memtag = NvBufSurfaceTag_VIDEO_ENC;
      cParams.memType = NVBUF_MEM_SURFACE_ARRAY;
      for (uint32_t i = 0; i < encoder_->output_plane.getNumBuffers(); i++) {
        ret = NvBufSurf::NvAllocate(&cParams, 1, &fd);
        INIT_ERROR(ret, "Failed to create NvBuffer");
        RTC_LOG(LS_ERROR) << "NvBufferCreateEx i:" << i << " fd:" << fd;
        output_plane_fd_[i] = fd;
      }
    } else {
      ret = encoder_->output_plane.setupPlane(V4L2_MEMORY_USERPTR, 1, false,
                                              false);
      INIT_ERROR(ret < 0, "Failed to setupPlane at encoder output_plane");
    }
  } else {
    ret = encoder_->output_plane.setupPlane(V4L2_MEMORY_MMAP, 1, true, false);
    INIT_ERROR(ret < 0, "Failed to setupPlane at encoder output_plane");
  }

  ret = encoder_->capture_plane.setupPlane(V4L2_MEMORY_MMAP, 1, true, false);
  INIT_ERROR(ret < 0, "Failed to setupPlane at capture_plane");

  ret = encoder_->subscribeEvent(V4L2_EVENT_EOS, 0, 0);
  INIT_ERROR(ret < 0, "Failed to subscribeEvent V4L2_EVENT_EOS");

  ret = encoder_->output_plane.setStreamStatus(true);
  INIT_ERROR(ret < 0, "Failed to setStreamStatus at encoder output_plane");

  ret = encoder_->capture_plane.setStreamStatus(true);
  INIT_ERROR(ret < 0, "Failed to setStreamStatus at encoder capture_plane");

  encoder_->capture_plane.setDQThreadCallback(EncodeFinishedCallbackFunction);
  encoder_->capture_plane.startDQThread(this);

  for (uint32_t i = 0; i < encoder_->capture_plane.getNumBuffers(); i++) {
    struct v4l2_buffer v4l2_buf;
    struct v4l2_plane planes[MAX_PLANES];
    memset(&v4l2_buf, 0, sizeof(v4l2_buf));
    memset(planes, 0, MAX_PLANES * sizeof(struct v4l2_plane));
    v4l2_buf.index = i;
    v4l2_buf.m.planes = planes;
    ret = encoder_->capture_plane.qBuffer(v4l2_buf, NULL);
    INIT_ERROR(ret < 0, "Failed to qBuffer at encoder capture_plane");
  }

  configured_framerate_ = framerate_;

  return WEBRTC_VIDEO_CODEC_OK;
}

void JetsonVideoEncoder::JetsonRelease() {
  if (!encoder_)
    return;
  SendEOS();
  encoder_->capture_plane.waitForDQThread(2000);
  encoder_->capture_plane.deinitPlane();
  if (use_dmabuff_) {
    for (uint32_t i = 0; i < encoder_->output_plane.getNumBuffers(); i++) {
      if (encoder_->output_plane.unmapOutputBuffers(i, output_plane_fd_[i]) <
          0) {
        RTC_LOG(LS_ERROR)
            << "Failed to unmapOutputBuffers at encoder output_plane";
      }
      if (NvBufSurf::NvDestroy(output_plane_fd_[i]) < 0) {
        RTC_LOG(LS_ERROR)
            << "Failed to NvBufferDestroy at encoder output_plane";
      }
    }
  } else {
    encoder_->output_plane.deinitPlane();
  }
  delete encoder_;
  encoder_ = nullptr;
}

void JetsonVideoEncoder::SendEOS() {
  if (encoder_->output_plane.getStreamStatus()) {
    struct v4l2_buffer v4l2_buf;
    struct v4l2_plane planes[MAX_PLANES];
    NvBuffer* buffer;

    memset(&v4l2_buf, 0, sizeof(v4l2_buf));
    memset(planes, 0, MAX_PLANES * sizeof(struct v4l2_plane));
    v4l2_buf.m.planes = planes;

    if (encoder_->output_plane.getNumQueuedBuffers() ==
        encoder_->output_plane.getNumBuffers()) {
      if (encoder_->output_plane.dqBuffer(v4l2_buf, &buffer, NULL, 10) < 0) {
        RTC_LOG(LS_ERROR) << "Failed to dqBuffer at encoder output_plane";
      }
    }
    planes[0].bytesused = 0;
    for (int i = 0; i < buffer->n_planes; i++) {
      buffer->planes[i].bytesused = 0;
    }
    if (encoder_->output_plane.qBuffer(v4l2_buf, NULL) < 0) {
      RTC_LOG(LS_ERROR) << "Failed to qBuffer at encoder output_plane";
    }
  }
}

bool JetsonVideoEncoder::EncodeFinishedCallbackFunction(
    struct v4l2_buffer* v4l2_buf,
    NvBuffer* buffer,
    NvBuffer* shared_buffer,
    void* data) {
  return ((JetsonVideoEncoder*)data)
      ->EncodeFinishedCallback(v4l2_buf, buffer, shared_buffer);
}

bool JetsonVideoEncoder::EncodeFinishedCallback(struct v4l2_buffer* v4l2_buf,
                                                NvBuffer* buffer,
                                                NvBuffer* shared_buffer) {
  if (!v4l2_buf) {
    RTC_LOG(LS_INFO) << __FUNCTION__ << " v4l2_buf is null";
    return false;
  }

  if (buffer->planes[0].bytesused == 0) {
    RTC_LOG(LS_INFO) << __FUNCTION__ << " buffer size is zero";
    return false;
  }

  uint64_t timestamp = v4l2_buf->timestamp.tv_sec * rtc::kNumMicrosecsPerSec +
                       v4l2_buf->timestamp.tv_usec;

  std::unique_ptr<FrameParams> params;
  {
    webrtc::MutexLock lock(&frame_params_lock_);
    do {
      if (frame_params_.empty()) {
        RTC_LOG(LS_WARNING)
            << __FUNCTION__
            << "Frame parameter is not found. SkipFrame timestamp:"
            << timestamp;
        return true;
      }
      params = std::move(frame_params_.front());
      frame_params_.pop();
    } while (params->timestamp_us < timestamp);
    if (params->timestamp_us != timestamp) {
      RTC_LOG(LS_WARNING)
          << __FUNCTION__
          << "Frame parameter is not found. SkipFrame timestamp:" << timestamp;
      return true;
    }
  }

  v4l2_ctrl_videoenc_outputbuf_metadata enc_metadata;
  if (encoder_->getMetadata(v4l2_buf->index, enc_metadata) != 0) {
    RTC_LOG(LS_WARNING) << __FUNCTION__
                        << "getMetadata failed. SkipFrame timestamp:"
                        << timestamp;
    return true;
  }

  SendFrame(buffer->planes[0].data, buffer->planes[0].bytesused,
            std::move(params), &enc_metadata);

  if (encoder_->capture_plane.qBuffer(*v4l2_buf, NULL) < 0) {
    RTC_LOG(LS_ERROR) << __FUNCTION__ << "Failed to qBuffer at capture_plane";
    return false;
  }

  return true;
}

int32_t JetsonVideoEncoder::RegisterEncodeCompleteCallback(
    webrtc::EncodedImageCallback* callback) {
  callback_ = callback;
  return WEBRTC_VIDEO_CODEC_OK;
}

void JetsonVideoEncoder::SetRates(const RateControlParameters& parameters) {
  if (encoder_ == nullptr)
    return;
  if (parameters.bitrate.get_sum_bps() <= 0 || parameters.framerate_fps <= 0)
    return;

  RTC_LOG(LS_INFO) << __FUNCTION__ << " framerate:" << parameters.framerate_fps
                   << " bitrate:" << parameters.bitrate.ToString();
  if (svc_controller_) {
    svc_controller_->OnRatesUpdated(parameters.bitrate);
  }

  framerate_ = parameters.framerate_fps;
  target_bitrate_bps_ = parameters.bitrate.get_sum_bps();

  bitrate_adjuster_->SetTargetBitrateBps(target_bitrate_bps_);
  return;
}

void JetsonVideoEncoder::SetFramerate(uint32_t framerate) {
  if (configured_framerate_ == framerate) {
    return;
  }
  RTC_LOG(LS_INFO) << __FUNCTION__ << " " << framerate << "fps";
  if (encoder_->setFrameRate(framerate, 1) < 0) {
    RTC_LOG(LS_ERROR) << "Failed to set bitrate";
    return;
  }
  configured_framerate_ = framerate;
}

void JetsonVideoEncoder::SetBitrateBps(uint32_t bitrate_bps) {
  if (bitrate_bps < 300000 || (configured_bitrate_bps_ == bitrate_bps &&
                               configured_framerate_ == framerate_)) {
    return;
  }
  configured_bitrate_bps_ = bitrate_bps;

  // VP9 の setBitrate は、設定されたフレームレートを見ずに
  // 60fps での bps を見てるっぽいので、ここで渡す bps を調整する
  if (codec_.codecType == webrtc::kVideoCodecVP9) {
    auto adjusted_bps = bitrate_bps * 60 / configured_framerate_;
    RTC_LOG(LS_INFO) << __FUNCTION__ << " bps=" << bitrate_bps
                     << " adjusted_bps=" << adjusted_bps;
    bitrate_bps = adjusted_bps;
  } else {
    RTC_LOG(LS_INFO) << __FUNCTION__ << " bps=" << bitrate_bps;
  }

  if (encoder_->setBitrate(bitrate_bps) < 0) {
    RTC_LOG(LS_ERROR) << "Failed to setBitrate";
    return;
  }
}

webrtc::VideoEncoder::EncoderInfo JetsonVideoEncoder::GetEncoderInfo() const {
  EncoderInfo info;
  info.supports_native_handle = true;
  info.implementation_name = "Jetson Video Encoder";
  if (codec_.codecType == webrtc::kVideoCodecH264) {
    static const int kLowH264QpThreshold = 34;
    static const int kHighH264QpThreshold = 40;
    info.scaling_settings = VideoEncoder::ScalingSettings(kLowH264QpThreshold,
                                                          kHighH264QpThreshold);
  } else if (codec_.codecType == webrtc::kVideoCodecVP8) {
    static const int kLowVp8QpThreshold = 29;
    static const int kHighVp8QpThreshold = 95;
    info.scaling_settings =
        VideoEncoder::ScalingSettings(kLowVp8QpThreshold, kHighVp8QpThreshold);
  } else if (codec_.codecType == webrtc::kVideoCodecVP9) {
    static const int kLowVp9QpThreshold = 150;
    static const int kHighVp9QpThreshold = 151;
    info.scaling_settings =
        VideoEncoder::ScalingSettings(kLowVp9QpThreshold, kHighVp9QpThreshold);
  } else if (codec_.codecType == webrtc::kVideoCodecAV1) {
    static const int kLowAv1QpThreshold = 145;
    static const int kHighAv1QpThreshold = 205;
    info.scaling_settings =
        VideoEncoder::ScalingSettings(kLowAv1QpThreshold, kHighAv1QpThreshold);
  }
  return info;
}

int32_t JetsonVideoEncoder::Encode(
    const webrtc::VideoFrame& input_frame,
    const std::vector<webrtc::VideoFrameType>* frame_types) {
  if (!callback_) {
    RTC_LOG(LS_WARNING)
        << "InitEncode() has been called, but a callback function "
        << "has not been set with RegisterEncodeCompleteCallback()";
    return WEBRTC_VIDEO_CODEC_UNINITIALIZED;
  }

  int fd = 0;
  webrtc::VideoType video_type;
  uint8_t* native_data;
  rtc::scoped_refptr<webrtc::VideoFrameBuffer> frame_buffer =
      input_frame.video_frame_buffer();
  std::shared_ptr<JetsonJpegDecoder> decoder;
  if (frame_buffer->type() == webrtc::VideoFrameBuffer::Type::kNative) {
    use_native_ = true;
    JetsonBuffer* jetson_buffer =
        static_cast<JetsonBuffer*>(frame_buffer.get());
    video_type = jetson_buffer->VideoType();
    raw_width_ = jetson_buffer->RawWidth();
    raw_height_ = jetson_buffer->RawHeight();
    use_dmabuff_ = true;
    fd = jetson_buffer->DecodedFd();
    decode_pixfmt_ = jetson_buffer->V4L2PixelFormat();
    decoder = jetson_buffer->JpegDecoder();
  } else {
    use_native_ = false;
  }

  if (encoder_ == nullptr) {
    if (JetsonConfigure() != WEBRTC_VIDEO_CODEC_OK) {
      RTC_LOG(LS_ERROR) << "Failed to JetsonConfigure";
      return WEBRTC_VIDEO_CODEC_ERROR;
    }
  }

  bool force_key_frame = false;
  if (frame_types != nullptr) {
    RTC_DCHECK_EQ(frame_types->size(), static_cast<size_t>(1));
    if ((*frame_types)[0] == webrtc::VideoFrameType::kEmptyFrame) {
      return WEBRTC_VIDEO_CODEC_OK;
    }
    if ((*frame_types)[0] == webrtc::VideoFrameType::kVideoFrameKey) {
      if (encoder_->forceIDR() < 0) {
        RTC_LOG(LS_ERROR) << "Failed to forceIDR";
      }
    }
  }

  SetFramerate(framerate_);
  SetBitrateBps(bitrate_adjuster_->GetAdjustedBitrateBps());
  {
    webrtc::MutexLock lock(&frame_params_lock_);
    frame_params_.push(absl::make_unique<FrameParams>(
        frame_buffer->width(), frame_buffer->height(),
        input_frame.render_time_ms(), input_frame.ntp_time_ms(),
        input_frame.timestamp_us(), input_frame.timestamp(),
        input_frame.rotation(), input_frame.color_space(), decoder));
  }

  struct v4l2_buffer v4l2_buf;
  struct v4l2_plane planes[MAX_PLANES];

  memset(&v4l2_buf, 0, sizeof(v4l2_buf));
  memset(planes, 0, sizeof(planes));
  v4l2_buf.m.planes = planes;

  if (use_native_) {
    NvBuffer* buffer;
    if (encoder_->output_plane.getNumQueuedBuffers() ==
        encoder_->output_plane.getNumBuffers()) {
      if (encoder_->output_plane.dqBuffer(v4l2_buf, &buffer, NULL, 10) < 0) {
        RTC_LOG(LS_ERROR) << "Failed to dqBuffer at encoder output_plane";
        return WEBRTC_VIDEO_CODEC_ERROR;
      }
    } else {
      buffer = encoder_->output_plane.getNthBuffer(
          encoder_->output_plane.getNumQueuedBuffers());
      v4l2_buf.index = encoder_->output_plane.getNumQueuedBuffers();
    }

    int src_dma_fd = -1;
    if (use_dmabuff_) {
      src_dma_fd = fd;
    } else if (video_type == webrtc::VideoType::kYUY2 ||
               video_type == webrtc::VideoType::kUYVY) {
      buffer->planes[0].bytesused = buffer->planes[0].fmt.width *
                                    buffer->planes[0].fmt.bytesperpixel *
                                    buffer->planes[0].fmt.height;
      buffer->planes[0].data = native_data;
    } else if (video_type == webrtc::VideoType::kI420) {
      size_t offset = 0;
      for (int i = 0; i < buffer->n_planes; i++) {
        buffer->planes[i].bytesused = buffer->planes[i].fmt.width *
                                      buffer->planes[i].fmt.bytesperpixel *
                                      buffer->planes[i].fmt.height;
        buffer->planes[i].data = native_data + offset;
        offset += buffer->planes[i].bytesused;
      }
    } else if (video_type == webrtc::VideoType::kYV12) {
      size_t offset = 0;
      buffer->planes[0].bytesused = buffer->planes[0].fmt.width *
                                    buffer->planes[0].fmt.bytesperpixel *
                                    buffer->planes[0].fmt.height;
      buffer->planes[0].data = native_data;
      offset += buffer->planes[0].bytesused;
      buffer->planes[2].bytesused = buffer->planes[1].fmt.width *
                                    buffer->planes[1].fmt.bytesperpixel *
                                    buffer->planes[1].fmt.height;
      buffer->planes[2].data = native_data + offset;
      offset += buffer->planes[2].bytesused;
      buffer->planes[1].bytesused = buffer->planes[2].fmt.width *
                                    buffer->planes[2].fmt.bytesperpixel *
                                    buffer->planes[2].fmt.height;
      buffer->planes[1].data = native_data + offset;
    } else {
      RTC_LOG(LS_ERROR) << "Unsupported webrtc::VideoType";
      return WEBRTC_VIDEO_CODEC_ERROR;
    }

    NvBufSurf::NvCommonTransformParams transform_params;
    /* Indicates which of the transform parameters are valid */
    memset(&transform_params, 0, sizeof(transform_params));
    transform_params.src_top = 0;
    transform_params.src_left = 0;
    transform_params.src_width = raw_width_;
    transform_params.src_height = raw_height_;
    transform_params.dst_top = 0;
    transform_params.dst_left = 0;
    transform_params.dst_width = width_;
    transform_params.dst_height = height_;
    transform_params.flag =
        (NvBufSurfTransform_Transform_Flag)(NVBUFSURF_TRANSFORM_FILTER |
                                            NVBUFSURF_TRANSFORM_CROP_SRC);
    transform_params.flip = NvBufSurfTransform_None;
    transform_params.filter = NvBufSurfTransformInter_Bilinear;
    if (NvBufSurf::NvTransform(&transform_params, src_dma_fd,
                               output_plane_fd_[v4l2_buf.index])) {
      RTC_LOG(LS_ERROR) << "Failed to NvBufferTransform";
      return WEBRTC_VIDEO_CODEC_ERROR;
    }

    planes[0].m.fd = output_plane_fd_[v4l2_buf.index];
    planes[0].bytesused = 1234;

    v4l2_buf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
    v4l2_buf.memory = V4L2_MEMORY_DMABUF;
    v4l2_buf.flags |= V4L2_BUF_FLAG_TIMESTAMP_COPY;
    v4l2_buf.timestamp.tv_sec =
        input_frame.timestamp_us() / rtc::kNumMicrosecsPerSec;
    v4l2_buf.timestamp.tv_usec =
        input_frame.timestamp_us() % rtc::kNumMicrosecsPerSec;

    if (encoder_->output_plane.qBuffer(v4l2_buf, nullptr) < 0) {
      RTC_LOG(LS_ERROR) << "Failed to qBuffer at converter output_plane";
      return WEBRTC_VIDEO_CODEC_ERROR;
    }
  } else {
    NvBuffer* buffer;

    RTC_LOG(LS_VERBOSE) << __FUNCTION__ << " output_plane.getNumBuffers: "
                        << encoder_->output_plane.getNumBuffers()
                        << " output_plane.getNumQueuedBuffers: "
                        << encoder_->output_plane.getNumQueuedBuffers();

    if (encoder_->output_plane.getNumQueuedBuffers() ==
        encoder_->output_plane.getNumBuffers()) {
      if (encoder_->output_plane.dqBuffer(v4l2_buf, &buffer, NULL, 10) < 0) {
        RTC_LOG(LS_ERROR) << "Failed to dqBuffer at encoder output_plane";
        return WEBRTC_VIDEO_CODEC_ERROR;
      }
    } else {
      buffer = encoder_->output_plane.getNthBuffer(
          encoder_->output_plane.getNumQueuedBuffers());
      v4l2_buf.index = encoder_->output_plane.getNumQueuedBuffers();
    }

    rtc::scoped_refptr<const webrtc::I420BufferInterface> i420_buffer =
        frame_buffer->ToI420();
    for (uint32_t i = 0; i < buffer->n_planes; i++) {
      const uint8_t* source_data;
      int source_stride;
      if (i == 0) {
        source_data = i420_buffer->DataY();
        source_stride = i420_buffer->StrideY();
      } else if (i == 1) {
        source_data = i420_buffer->DataU();
        source_stride = i420_buffer->StrideU();
      } else if (i == 2) {
        source_data = i420_buffer->DataV();
        source_stride = i420_buffer->StrideV();
      } else {
        break;
      }
      NvBuffer::NvBufferPlane& plane = buffer->planes[i];
      std::streamsize bytes_to_read = plane.fmt.bytesperpixel * plane.fmt.width;
      uint8_t* input_data = plane.data;
      plane.bytesused = 0;
      for (uint32_t j = 0; j < plane.fmt.height; j++) {
        memcpy(input_data, source_data + (source_stride * j), bytes_to_read);
        input_data += plane.fmt.stride;
      }
      plane.bytesused = plane.fmt.stride * plane.fmt.height;
    }

    v4l2_buf.flags |= V4L2_BUF_FLAG_TIMESTAMP_COPY;
    v4l2_buf.timestamp.tv_sec =
        input_frame.timestamp_us() / rtc::kNumMicrosecsPerSec;
    v4l2_buf.timestamp.tv_usec =
        input_frame.timestamp_us() % rtc::kNumMicrosecsPerSec;

    for (int i = 0; i < MAX_PLANES; i++) {
      NvBufSurface* surf = 0;
      if (NvBufSurfaceFromFd(buffer->planes[i].fd, (void**)(&surf)) == -1) {
        RTC_LOG(LS_ERROR) << __FUNCTION__ << "Failed to NvBufSurfaceFromFd";
        return WEBRTC_VIDEO_CODEC_ERROR;
      }

      if (NvBufSurfaceSyncForDevice(surf, 0, i) == -1) {
        RTC_LOG(LS_ERROR) << "Failed to NvBufSurfaceSyncForDevice";
        return WEBRTC_VIDEO_CODEC_ERROR;
      }
    }

    if (encoder_->output_plane.qBuffer(v4l2_buf, nullptr) < 0) {
      RTC_LOG(LS_ERROR) << "Failed to qBuffer at encoder output_plane";
      return WEBRTC_VIDEO_CODEC_ERROR;
    }
  }

  return WEBRTC_VIDEO_CODEC_OK;
}

int32_t JetsonVideoEncoder::SendFrame(
    unsigned char* buffer,
    size_t size,
    std::unique_ptr<FrameParams> params,
    v4l2_ctrl_videoenc_outputbuf_metadata* enc_metadata) {
  if (!callback_) {
    RTC_LOG(LS_WARNING)
        << "InitEncode() has been called, but a callback function "
        << "has not been set with RegisterEncodeCompleteCallback()";
    return WEBRTC_VIDEO_CODEC_UNINITIALIZED;
  }

  encoded_image_.SetRtpTimestamp(params->timestamp_rtp);
  encoded_image_.SetColorSpace(params->color_space);
  encoded_image_._encodedWidth = params->width;
  encoded_image_._encodedHeight = params->height;
  encoded_image_.capture_time_ms_ = params->render_time_ms;
  encoded_image_.ntp_time_ms_ = params->ntp_time_ms;
  encoded_image_.rotation_ = params->rotation;
  encoded_image_.qp_ = enc_metadata->AvgQP;
  if (enc_metadata->KeyFrame) {
    encoded_image_._frameType = webrtc::VideoFrameType::kVideoFrameKey;
  } else {
    encoded_image_._frameType = webrtc::VideoFrameType::kVideoFrameDelta;
  }

  webrtc::CodecSpecificInfo codec_specific;
  codec_specific.codecType = codec_.codecType;
  if (codec_.codecType == webrtc::kVideoCodecH264) {
    auto encoded_image_buffer =
        webrtc::EncodedImageBuffer::Create(buffer, size);
    encoded_image_.SetEncodedData(encoded_image_buffer);

    codec_specific.codecSpecific.H264.packetization_mode =
        webrtc::H264PacketizationMode::NonInterleaved;
  } else if (codec_.codecType == webrtc::kVideoCodecAV1 ||
             codec_.codecType == webrtc::kVideoCodecVP9 ||
             codec_.codecType == webrtc::kVideoCodecVP8) {
    // VP8, VP9, AV1 はIVFヘッダーがエンコードフレームについているので取り除く
    if ((buffer[0] == 'D') && (buffer[1] == 'K') && (buffer[2] == 'I') &&
        (buffer[3] == 'F')) {
      buffer += 32;
      size -= 32;
    }
    buffer += 12;
    size -= 12;

    auto encoded_image_buffer =
        webrtc::EncodedImageBuffer::Create(buffer, size);
    encoded_image_.SetEncodedData(encoded_image_buffer);

    if (codec_.codecType == webrtc::kVideoCodecVP8) {
      codec_specific.codecSpecific.VP8.keyIdx = webrtc::kNoKeyIdx;
      // nonReference かを知ることはできなかった
      codec_specific.codecSpecific.VP8.nonReference = false;
    } else if (codec_.codecType == webrtc::kVideoCodecVP9) {
      if (enc_metadata->KeyFrame) {
        gof_idx_ = 0;
      }
      codec_specific.codecSpecific.VP9.inter_pic_predicted =
          enc_metadata->KeyFrame ? false : true;
      codec_specific.codecSpecific.VP9.flexible_mode = false;
      codec_specific.codecSpecific.VP9.ss_data_available =
          enc_metadata->KeyFrame ? true : false;
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
    } else if (codec_.codecType == webrtc::kVideoCodecAV1) {
      bool is_key = buffer[2] == 0x0a;
      // v4l2_ctrl_videoenc_outputbuf_metadata.KeyFrame が効いていない
      // キーフレームの時には OBU_SEQUENCE_HEADER が入っているために 0x0a になるためこれを使う
      // キーフレームではない時には OBU_FRAME が入っていて 0x32 になっている
      if (is_key) {
        encoded_image_._frameType = webrtc::VideoFrameType::kVideoFrameKey;
      }

      std::vector<webrtc::ScalableVideoController::LayerFrameConfig>
          layer_frames = svc_controller_->NextFrameConfig(is_key);
      codec_specific.end_of_picture = true;
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
  }

  RTC_LOG(LS_VERBOSE) << "key_frame=" << enc_metadata->KeyFrame
                      << " size=" << size << " qp=" << encoded_image_.qp_;

  webrtc::EncodedImageCallback::Result result =
      callback_->OnEncodedImage(encoded_image_, &codec_specific);
  if (result.error != webrtc::EncodedImageCallback::Result::OK) {
    RTC_LOG(LS_ERROR) << __FUNCTION__
                      << " OnEncodedImage failed error:" << result.error;
    return WEBRTC_VIDEO_CODEC_ERROR;
  }
  bitrate_adjuster_->Update(size);
  return WEBRTC_VIDEO_CODEC_OK;
}
