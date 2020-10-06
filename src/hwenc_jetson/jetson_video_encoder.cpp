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
#include <modules/video_coding/utility/vp8_header_parser.h>
#include <modules/video_coding/utility/vp9_uncompressed_header_parser.h>
#include <rtc_base/checks.h>
#include <rtc_base/logging.h>
#include <rtc_base/time_utils.h>
#include <system_wrappers/include/metrics.h>
#include <third_party/libyuv/include/libyuv/convert.h>
#include <third_party/libyuv/include/libyuv/convert_from.h>
#include <third_party/libyuv/include/libyuv/video_common.h>

// L4T Multimedia API
#include <nvbuf_utils.h>

#include "jetson_buffer.h"

#define H264HWENC_HEADER_DEBUG 0
#define INIT_ERROR(cond, desc)                 \
  if (cond) {                                  \
    RTC_LOG(LS_ERROR) << __FUNCTION__ << desc; \
    Release();                                 \
    return WEBRTC_VIDEO_CODEC_ERROR;           \
  }

namespace {
struct nal_entry {
  size_t offset;
  size_t size;
};

}  // namespace

JetsonVideoEncoder::JetsonVideoEncoder(const cricket::VideoCodec& codec)
    : callback_(nullptr),
      converter_(nullptr),
      encoder_(nullptr),
      configured_framerate_(30),
      use_native_(false),
      use_dmabuff_(false) {}

JetsonVideoEncoder::~JetsonVideoEncoder() {
  Release();
}

bool JetsonVideoEncoder::IsSupportedVP8() {
  auto encoder = NvVideoEncoder::createVideoEncoder("enc0");
  auto ret = encoder->setCapturePlaneFormat(V4L2_PIX_FMT_VP8, 1024, 768,
                                            2 * 1024 * 1024);
  return ret >= 0;
}

bool JetsonVideoEncoder::IsSupportedVP9() {
  auto encoder = NvVideoEncoder::createVideoEncoder("enc0");
  auto ret = encoder->setCapturePlaneFormat(V4L2_PIX_FMT_VP9, 1024, 768,
                                            2 * 1024 * 1024);
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
    RTC_LOG(LS_INFO) << "complexity: " << (int)codec_settings->VP9().complexity;
    RTC_LOG(LS_INFO) << "numberOfTemporalLayers: "
                     << codec_settings->VP9().numberOfTemporalLayers;
    RTC_LOG(LS_INFO) << "denoisingOn: " << codec_settings->VP9().denoisingOn;
    RTC_LOG(LS_INFO) << "frameDroppingOn: "
                     << codec_settings->VP9().frameDroppingOn;
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
  }
  framerate_ = codec_settings->maxFramerate;

  RTC_LOG(LS_INFO) << "InitEncode " << framerate_ << "fps "
                   << target_bitrate_bps_ << "bit/sec　"
                   << codec_settings->maxBitrate << "kbit/sec　";

  // Initialize encoded image. Default buffer size: size of unencoded data.
  encoded_image_._completeFrame = true;
  encoded_image_._encodedWidth = 0;
  encoded_image_._encodedHeight = 0;
  encoded_image_.set_size(0);
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
  bool use_converter = use_native_ &&
                       (width_ != raw_width_ ||
                        height_ != raw_height_ ||
                        decode_pixfmt_ != V4L2_PIX_FMT_YUV420M);

  if (use_converter) {
    enc0_buffer_queue_ = new std::queue<NvBuffer*>;

    converter_ = NvVideoConverter::createVideoConverter("conv");
    INIT_ERROR(!converter_, "Failed to createVideoConverter");

    ret = converter_->setOutputPlaneFormat(
        decode_pixfmt_, raw_width_, raw_height_, V4L2_NV_BUFFER_LAYOUT_PITCH);
    INIT_ERROR(ret < 0, "Failed to converter setOutputPlaneFormat");

    ret = converter_->setCapturePlaneFormat(
        V4L2_PIX_FMT_YUV420M, width_, height_, V4L2_NV_BUFFER_LAYOUT_BLOCKLINEAR);
    INIT_ERROR(ret < 0, "Failed to converter setCapturePlaneFormat");

    ret = converter_->setCropRect(0, 0, raw_width_, raw_height_);
    INIT_ERROR(ret < 0, "Failed to converter setCropRect");

    if (use_dmabuff_) {
      ret = converter_->output_plane.setupPlane(V4L2_MEMORY_DMABUF, 1, false,
                                                false);
      INIT_ERROR(ret < 0, "Failed to setupPlane at converter output_plane");
    } else {
      ret = converter_->output_plane.setupPlane(V4L2_MEMORY_USERPTR, 1, false,
                                                false);
      INIT_ERROR(ret < 0, "Failed to setupPlane at converter output_plane");
    }
    
    NvBufferCreateParams create_params = {0};
    create_params.width = width_;
    create_params.height = height_;
    create_params.layout = NvBufferLayout_BlockLinear;
    create_params.payloadType = NvBufferPayload_SurfArray;
    create_params.colorFormat = NvBufferColorFormat_YUV420;
    create_params.nvbuf_tag = NvBufferTag_VIDEO_ENC;
    for (int i = 0; i < CONVERTER_CAPTURE_NUM; i++)
    {
      ret = NvBufferCreateEx(&dmabuff_fd_[i], &create_params);
      INIT_ERROR(ret < 0, "Failed to NvBufferCreateEx at converter");
    }
    
    ret = converter_->capture_plane.reqbufs(V4L2_MEMORY_DMABUF, CONVERTER_CAPTURE_NUM);
    INIT_ERROR(ret < 0, "Failed to reqbufs at converter capture_plane");

    ret = converter_->output_plane.setStreamStatus(true);
    INIT_ERROR(ret < 0, "Failed to setStreamStatus at converter output_plane");

    ret = converter_->capture_plane.setStreamStatus(true);
    INIT_ERROR(ret < 0, "Failed to setStreamStatus at converter capture_plane");

    converter_->capture_plane.setDQThreadCallback(
        ConvertFinishedCallbackFunction);
  }

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
      ret =
          encoder_->output_plane.setupPlane(V4L2_MEMORY_DMABUF, 1, false, false);
      INIT_ERROR(ret < 0, "Failed to setupPlane at encoder output_plane");
    } else {
      ret =
          encoder_->output_plane.setupPlane(V4L2_MEMORY_USERPTR, 1, false, false);
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

  if (use_converter) {
    converter_->capture_plane.startDQThread(this);

    for (uint32_t i = 0; i < CONVERTER_CAPTURE_NUM; i++) {
      struct v4l2_buffer v4l2_buf;
      struct v4l2_plane planes[MAX_PLANES];

      memset(&v4l2_buf, 0, sizeof(v4l2_buf));
      memset(planes, 0, MAX_PLANES * sizeof(struct v4l2_plane));

      v4l2_buf.index = i;
      v4l2_buf.m.planes = planes;
      v4l2_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      v4l2_buf.memory = V4L2_MEMORY_DMABUF;
      v4l2_buf.m.planes[0].m.fd = dmabuff_fd_[i];
      ret = converter_->capture_plane.qBuffer(v4l2_buf, nullptr);
      INIT_ERROR(ret < 0, "Failed to qBuffer at converter capture_plane");
    }

    for (uint32_t i = 0; i < encoder_->output_plane.getNumBuffers(); i++) {
      enc0_buffer_queue_->push(encoder_->output_plane.getNthBuffer(i));
    }
    encoder_->output_plane.setDQThreadCallback(EncodeOutputCallbackFunction);
    encoder_->output_plane.startDQThread(this);

    native_input_elem_ = converter_;
  } else {
    native_input_elem_ = encoder_;
  }
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
  if (converter_) {
    SendEOS(converter_);
  } else {
    SendEOS(encoder_);
  }
  encoder_->capture_plane.waitForDQThread(2000);
  encoder_->capture_plane.deinitPlane();
  encoder_->output_plane.deinitPlane();
  if (converter_) {
    delete enc0_buffer_queue_;
  }
  delete encoder_;
  encoder_ = nullptr;
  if (converter_) {
    converter_->capture_plane.waitForDQThread(2000);
    for(int i = 0; i < CONVERTER_CAPTURE_NUM; i++)
    {
      if(dmabuff_fd_[i] != 0)
      {
        NvBufferDestroy(dmabuff_fd_[i]);
      }
    }
    delete converter_;
    converter_ = nullptr;
  }
}

void JetsonVideoEncoder::SendEOS(NvV4l2Element* element) {
  if (element->output_plane.getStreamStatus()) {
    struct v4l2_buffer v4l2_buf;
    struct v4l2_plane planes[MAX_PLANES];
    NvBuffer* buffer;

    memset(&v4l2_buf, 0, sizeof(v4l2_buf));
    memset(planes, 0, MAX_PLANES * sizeof(struct v4l2_plane));
    v4l2_buf.m.planes = planes;

    if (element->output_plane.getNumQueuedBuffers() ==
        element->output_plane.getNumBuffers()) {
      if (element->output_plane.dqBuffer(v4l2_buf, &buffer, NULL, 10) < 0) {
        RTC_LOG(LS_ERROR) << "Failed to dqBuffer at encoder output_plane";
      }
    }
    planes[0].bytesused = 0;
    for (int i = 0; i < buffer->n_planes; i++) {
      buffer->planes[i].bytesused = 0;
    }
    if (element->output_plane.qBuffer(v4l2_buf, NULL) < 0) {
      RTC_LOG(LS_ERROR) << "Failed to qBuffer at encoder output_plane";
    }
  }
}

bool JetsonVideoEncoder::ConvertFinishedCallbackFunction(
    struct v4l2_buffer* v4l2_buf,
    NvBuffer* buffer,
    NvBuffer* shared_buffer,
    void* data) {
  return ((JetsonVideoEncoder*)data)
      ->ConvertFinishedCallback(v4l2_buf, buffer, shared_buffer);
}

bool JetsonVideoEncoder::ConvertFinishedCallback(struct v4l2_buffer* v4l2_buf,
                                                 NvBuffer* buffer,
                                                 NvBuffer* shared_buffer) {
  NvBuffer* enc0_buffer;
  struct v4l2_buffer enc0_qbuf;
  struct v4l2_plane planes[MAX_PLANES];

  if (!v4l2_buf) {
    RTC_LOG(LS_ERROR) << __FUNCTION__ << " v4l2_buf is null";
    return false;
  }
  
  {
    std::unique_lock<std::mutex> lock(enc0_buffer_mtx_);
    enc0_buffer_cond_.wait(lock, [this] { return !enc0_buffer_queue_->empty(); });
    enc0_buffer = enc0_buffer_queue_->front();
    enc0_buffer_queue_->pop();
  }

  memset(&enc0_qbuf, 0, sizeof(enc0_qbuf));
  memset(&planes, 0, sizeof(planes));

  enc0_qbuf.index = enc0_buffer->index;
  enc0_qbuf.m.planes = planes;
  buffer->planes[0].fd = dmabuff_fd_[v4l2_buf->index];

  enc0_qbuf.flags |= V4L2_BUF_FLAG_TIMESTAMP_COPY;
  enc0_qbuf.timestamp.tv_sec = v4l2_buf->timestamp.tv_sec;
  enc0_qbuf.timestamp.tv_usec = v4l2_buf->timestamp.tv_usec;

  if (encoder_->output_plane.qBuffer(enc0_qbuf, buffer) < 0) {
    RTC_LOG(LS_ERROR) << __FUNCTION__
                      << " Failed to qBuffer at encoder output_plane";
    return false;
  }

  if (v4l2_buf->m.planes[0].bytesused == 0) {
    RTC_LOG(LS_ERROR) << __FUNCTION__ << " buffer size is zero";
    return false;
  }

  return true;
}

bool JetsonVideoEncoder::EncodeOutputCallbackFunction(
    struct v4l2_buffer* v4l2_buf,
    NvBuffer* buffer,
    NvBuffer* shared_buffer,
    void* data) {
  return ((JetsonVideoEncoder*)data)
      ->EncodeOutputCallback(v4l2_buf, buffer, shared_buffer);
}

bool JetsonVideoEncoder::EncodeOutputCallback(struct v4l2_buffer* v4l2_buf,
                                              NvBuffer* buffer,
                                              NvBuffer* shared_buffer) {
  struct v4l2_buffer conv_qbuf;
  struct v4l2_plane planes[MAX_PLANES];

  if (!v4l2_buf) {
    RTC_LOG(LS_INFO) << __FUNCTION__ << " v4l2_buf is null";
    return false;
  }

  memset(&conv_qbuf, 0, sizeof(conv_qbuf));
  memset(&planes, 0, sizeof(planes));

  conv_qbuf.index = shared_buffer->index;
  conv_qbuf.m.planes = planes;
  conv_qbuf.m.planes[0].m.fd = dmabuff_fd_[shared_buffer->index];
  {
    std::unique_lock<std::mutex> lock(enc0_buffer_mtx_);
    enc0_buffer_queue_->push(buffer);

    if (converter_->capture_plane.qBuffer(conv_qbuf, nullptr) < 0) {
      RTC_LOG(LS_ERROR) << __FUNCTION__
                        << "Failed to qBuffer at converter capture_plane";
      return false;
    }

    enc0_buffer_cond_.notify_all();
  }

  return true;
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

  v4l2_ctrl_videoenc_outputbuf_metadata enc_metadata;
  if (encoder_->getMetadata(v4l2_buf->index, enc_metadata) == 0) {
    if (enc_metadata.KeyFrame) {
      encoded_image_._frameType = webrtc::VideoFrameType::kVideoFrameKey;
    } else {
      encoded_image_._frameType = webrtc::VideoFrameType::kVideoFrameDelta;
    }
  }

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

  encoded_image_._encodedWidth = params->width;
  encoded_image_._encodedHeight = params->height;
  encoded_image_.capture_time_ms_ = params->render_time_ms;
  encoded_image_.ntp_time_ms_ = params->ntp_time_ms;
  encoded_image_.SetTimestamp(params->timestamp_rtp);
  encoded_image_.rotation_ = params->rotation;
  encoded_image_.SetColorSpace(params->color_space);

  SendFrame(buffer->planes[0].data, buffer->planes[0].bytesused);

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
  }
  info.has_internal_source = false;
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
  std::shared_ptr<NvJPEGDecoder> decoder;
  if (frame_buffer->type() == webrtc::VideoFrameBuffer::Type::kNative) {
    use_native_ = true;
    JetsonBuffer* jetson_buffer =
        static_cast<JetsonBuffer*>(frame_buffer.get());
    video_type = jetson_buffer->VideoType();
    raw_width_ = jetson_buffer->RawWidth();
    raw_height_ = jetson_buffer->RawHeight();
    if (video_type == webrtc::VideoType::kMJPEG) {
      use_dmabuff_ = true;
      fd = jetson_buffer->DecodedFd();
      decode_pixfmt_ = jetson_buffer->V4L2PixelFormat();
      decoder = jetson_buffer->JpegDecoder();
    } else {
      use_dmabuff_ = false;
      if (video_type == webrtc::VideoType::kYUY2) {
        decode_pixfmt_ = V4L2_PIX_FMT_YUYV;
      } else if (video_type == webrtc::VideoType::kI420) {
        decode_pixfmt_ = V4L2_PIX_FMT_YUV420M;
      } else if (video_type == webrtc::VideoType::kYV12) {
        decode_pixfmt_ = V4L2_PIX_FMT_YUV420M;
      } else if (video_type == webrtc::VideoType::kNV12) {
        decode_pixfmt_ = V4L2_PIX_FMT_NV12M;
      } else if (video_type == webrtc::VideoType::kUYVY) {
        decode_pixfmt_ = V4L2_PIX_FMT_UYVY;
      } else {
        RTC_LOG(LS_ERROR) << "Unsupported VideoType";
        return WEBRTC_VIDEO_CODEC_ERROR;
      }
      native_data = jetson_buffer->Data();
    }
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
        input_frame.rotation(), input_frame.color_space(),
        decoder));
  }

  struct v4l2_buffer v4l2_buf;
  struct v4l2_plane planes[MAX_PLANES];

  memset(&v4l2_buf, 0, sizeof(v4l2_buf));
  memset(planes, 0, sizeof(planes));
  v4l2_buf.m.planes = planes;

  if (use_native_) {
    NvBuffer* buffer;
    if (native_input_elem_->output_plane.getNumQueuedBuffers() ==
        native_input_elem_->output_plane.getNumBuffers()) {
      if (native_input_elem_->output_plane.dqBuffer(v4l2_buf, &buffer, NULL, 10) < 0) {
        RTC_LOG(LS_ERROR) << "Failed to dqBuffer at converter output_plane";
        return WEBRTC_VIDEO_CODEC_ERROR;
      }
    } else if (!use_dmabuff_ || native_input_elem_ == encoder_) {
      buffer = native_input_elem_->output_plane.getNthBuffer(
          native_input_elem_->output_plane.getNumQueuedBuffers());
      v4l2_buf.index = native_input_elem_->output_plane.getNumQueuedBuffers();
    }

    if (use_dmabuff_) {
      planes[0].m.fd = fd;
      planes[0].bytesused = 1234;
    } else if (video_type == webrtc::VideoType::kYUY2 ||
               video_type == webrtc::VideoType::kUYVY) {
      buffer->planes[0].bytesused = buffer->planes[0].fmt.width *
                            buffer->planes[0].fmt.bytesperpixel *
                            buffer->planes[0].fmt.height;
      buffer->planes[0].data = native_data;
    } else if (video_type == webrtc::VideoType::kI420 ||
               video_type == webrtc::VideoType::kNV12) {
      size_t offset = 0;
      for (int i = 0; i < buffer->n_planes; i++)
      {
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

    v4l2_buf.flags |= V4L2_BUF_FLAG_TIMESTAMP_COPY;
    v4l2_buf.timestamp.tv_sec =
        input_frame.timestamp_us() / rtc::kNumMicrosecsPerSec;
    v4l2_buf.timestamp.tv_usec =
        input_frame.timestamp_us() % rtc::kNumMicrosecsPerSec;

    if (native_input_elem_->output_plane.qBuffer(v4l2_buf, nullptr) < 0) {
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
      if (NvBufferMemSyncForDevice(buffer->planes[i].fd, i,
                                   (void**)&buffer->planes[i].data) < 0) {
        RTC_LOG(LS_ERROR) << "Failed to NvBufferMemSyncForDevice";
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

int32_t JetsonVideoEncoder::SendFrame(unsigned char* buffer, size_t size) {
  webrtc::RTPFragmentationHeader frag_header;
  webrtc::CodecSpecificInfo codec_specific;

  if (codec_.codecType == webrtc::kVideoCodecH264) {
    sending_encoded_image_.reset(new webrtc::EncodedImage(buffer, size, size));
    sending_encoded_image_->_frameType =
        webrtc::VideoFrameType::kVideoFrameDelta;
    sending_encoded_image_->_completeFrame = encoded_image_._completeFrame;
    sending_encoded_image_->_encodedWidth = encoded_image_._encodedWidth;
    sending_encoded_image_->_encodedHeight = encoded_image_._encodedHeight;
    sending_encoded_image_->timing_.flags = encoded_image_.timing_.flags;
    sending_encoded_image_->content_type_ = encoded_image_.content_type_;
    sending_encoded_image_->capture_time_ms_ = encoded_image_.capture_time_ms_;
    sending_encoded_image_->ntp_time_ms_ = encoded_image_.ntp_time_ms_;
    sending_encoded_image_->SetTimestamp(encoded_image_.Timestamp());
    sending_encoded_image_->rotation_ = encoded_image_.rotation_;
    if (encoded_image_.ColorSpace() != nullptr) {
      sending_encoded_image_->SetColorSpace(*encoded_image_.ColorSpace());
    }

    uint8_t zero_count = 0;
    size_t nal_start_idx = 0;
    std::vector<nal_entry> nals;
    for (size_t i = 0; i < size; i++) {
      uint8_t data = buffer[i];
      if ((i != 0) && (i == nal_start_idx)) {
        if ((data & 0x1F) == 0x05) {
          sending_encoded_image_->_frameType =
              webrtc::VideoFrameType::kVideoFrameKey;
        }
      }
      if (data == 0x01 && zero_count >= 2) {
        if (nal_start_idx != 0) {
          nals.push_back({nal_start_idx,
                          i - nal_start_idx + 1 - (zero_count == 2 ? 3 : 4)});
        }
        nal_start_idx = i + 1;
      }
      if (data == 0x00) {
        zero_count++;
      } else {
        zero_count = 0;
      }
    }
    if (nal_start_idx != 0) {
      nals.push_back({nal_start_idx, size - nal_start_idx});
    }

    frag_header.VerifyAndAllocateFragmentationHeader(nals.size());
    for (size_t i = 0; i < nals.size(); i++) {
      frag_header.fragmentationOffset[i] = nals[i].offset;
      frag_header.fragmentationLength[i] = nals[i].size;
    }

    codec_specific.codecType = webrtc::kVideoCodecH264;
    codec_specific.codecSpecific.H264.packetization_mode =
        webrtc::H264PacketizationMode::NonInterleaved;

    h264_bitstream_parser_.ParseBitstream(buffer, size);
    h264_bitstream_parser_.GetLastSliceQp(&encoded_image_.qp_);
    RTC_LOG(LS_VERBOSE) << __FUNCTION__
                        << " last slice qp:" << encoded_image_.qp_;
    sending_encoded_image_->qp_ = encoded_image_.qp_;
  } else if (codec_.codecType == webrtc::kVideoCodecVP9 ||
             codec_.codecType == webrtc::kVideoCodecVP8) {
    // VP8, VP9 はIVFヘッダーがエンコードフレームについているので取り除く
    if ((buffer[0] == 'D') && (buffer[1] == 'K') && (buffer[2] == 'I') &&
        (buffer[3] == 'F')) {
      buffer += 32;
      size -= 32;
    }
    buffer += 12;
    size -= 12;

    sending_encoded_image_.reset(new webrtc::EncodedImage(buffer, size, size));
    sending_encoded_image_->_frameType = encoded_image_._frameType;
    sending_encoded_image_->_completeFrame = encoded_image_._completeFrame;
    sending_encoded_image_->_encodedWidth = encoded_image_._encodedWidth;
    sending_encoded_image_->_encodedHeight = encoded_image_._encodedHeight;
    sending_encoded_image_->timing_.flags = encoded_image_.timing_.flags;
    sending_encoded_image_->content_type_ = encoded_image_.content_type_;
    sending_encoded_image_->capture_time_ms_ = encoded_image_.capture_time_ms_;
    sending_encoded_image_->ntp_time_ms_ = encoded_image_.ntp_time_ms_;
    sending_encoded_image_->SetTimestamp(encoded_image_.Timestamp());
    sending_encoded_image_->rotation_ = encoded_image_.rotation_;
    if (encoded_image_.ColorSpace() != nullptr) {
      sending_encoded_image_->SetColorSpace(*encoded_image_.ColorSpace());
    }
    const bool key_frame =
        encoded_image_._frameType == webrtc::VideoFrameType::kVideoFrameKey;

    frag_header.VerifyAndAllocateFragmentationHeader(1);
    frag_header.fragmentationOffset[0] = 0;
    frag_header.fragmentationLength[0] = size;

    if (key_frame) {
      gof_idx_ = 0;
    }
    codec_specific.codecType = codec_.codecType;
    if (codec_.codecType == webrtc::kVideoCodecVP8) {
      webrtc::vp8::GetQp(buffer, size, &encoded_image_.qp_);
      sending_encoded_image_->qp_ = encoded_image_.qp_;
      codec_specific.codecSpecific.VP8.keyIdx = webrtc::kNoKeyIdx;
      // nonReference かを知ることはできなかった
      codec_specific.codecSpecific.VP8.nonReference = false;
    } else if (codec_.codecType == webrtc::kVideoCodecVP9) {
      webrtc::vp9::GetQp(buffer, size, &encoded_image_.qp_);
      sending_encoded_image_->qp_ = encoded_image_.qp_;
      codec_specific.codecSpecific.VP9.inter_pic_predicted =
          key_frame ? false : true;
      codec_specific.codecSpecific.VP9.flexible_mode = false;
      codec_specific.codecSpecific.VP9.ss_data_available =
          key_frame ? true : false;
      codec_specific.codecSpecific.VP9.temporal_idx = webrtc::kNoTemporalIdx;
      codec_specific.codecSpecific.VP9.temporal_up_switch = true;
      codec_specific.codecSpecific.VP9.inter_layer_predicted = false;
      codec_specific.codecSpecific.VP9.gof_idx =
          static_cast<uint8_t>(gof_idx_++ % gof_.num_frames_in_gof);
      codec_specific.codecSpecific.VP9.num_spatial_layers = 1;
      codec_specific.codecSpecific.VP9.first_frame_in_picture = true;
      codec_specific.codecSpecific.VP9.end_of_picture = true;
      codec_specific.codecSpecific.VP9.spatial_layer_resolution_present = false;
      if (codec_specific.codecSpecific.VP9.ss_data_available) {
        codec_specific.codecSpecific.VP9.spatial_layer_resolution_present = true;
        codec_specific.codecSpecific.VP9.width[0] = encoded_image_._encodedWidth;
        codec_specific.codecSpecific.VP9.height[0] =
            encoded_image_._encodedHeight;
        codec_specific.codecSpecific.VP9.gof.CopyGofInfoVP9(gof_);
      }
    }
    RTC_LOG(LS_ERROR) << "key_frame=" << key_frame << " size=" << size
                        << " qp=" << encoded_image_.qp_;
  }

  webrtc::EncodedImageCallback::Result result = callback_->OnEncodedImage(
      *sending_encoded_image_, &codec_specific, &frag_header);
  if (result.error != webrtc::EncodedImageCallback::Result::OK) {
    RTC_LOG(LS_ERROR) << __FUNCTION__
                      << " OnEncodedImage failed error:" << result.error;
    return WEBRTC_VIDEO_CODEC_ERROR;
  }
  bitrate_adjuster_->Update(size);
  return WEBRTC_VIDEO_CODEC_OK;
}
