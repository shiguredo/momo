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

#include "mmal_h264_encoder.h"

#include <limits>
#include <string>

#include "common_video/libyuv/include/webrtc_libyuv.h"
#include "rtc/native_buffer.h"
#include "rtc_base/checks.h"
#include "rtc_base/logging.h"
#include "system_wrappers/include/metrics.h"
#include "third_party/libyuv/include/libyuv/convert.h"
#include "third_party/libyuv/include/libyuv/convert_from.h"
#include "third_party/libyuv/include/libyuv/video_common.h"

#define H264HWENC_HEADER_DEBUG 0
#define ROUND_UP_4(num) (((num) + 3) & ~3)

namespace {
struct nal_entry {
  size_t offset;
  size_t size;
};

const int kLowH264QpThreshold = 34;
const int kHighH264QpThreshold = 40;

int I420DataSize(const webrtc::I420BufferInterface& frame_buffer) {
  return frame_buffer.StrideY() * frame_buffer.height() +
         (frame_buffer.StrideU() + frame_buffer.StrideV()) *
             ((frame_buffer.height() + 1) / 2);
}

}  // namespace

MMALH264Encoder::MMALH264Encoder(const cricket::VideoCodec& codec)
    : callback_(nullptr),
      decoder_(nullptr),
      resizer_(nullptr),
      encoder_(nullptr),
      conn1_(nullptr),
      conn2_(nullptr),
      pool_out_(nullptr),
      bitrate_adjuster_(.5, .95),
      target_framerate_fps_(30),
      configured_framerate_fps_(30),
      configured_width_(0),
      configured_height_(0),
      use_native_(false),
      input_video_type_(webrtc::VideoType::kI420),
      encoded_buffer_length_(0) {}

MMALH264Encoder::~MMALH264Encoder() {}

int32_t MMALH264Encoder::InitEncode(const webrtc::VideoCodec* codec_settings,
                                    int32_t number_of_cores,
                                    size_t max_payload_size) {
  RTC_DCHECK(codec_settings);
  RTC_DCHECK_EQ(codec_settings->codecType, webrtc::kVideoCodecH264);

  int32_t release_ret = Release();
  if (release_ret != WEBRTC_VIDEO_CODEC_OK) {
    return release_ret;
  }

  bcm_host_init();

  width_ = codec_settings->width;
  height_ = codec_settings->height;
  target_bitrate_bps_ = codec_settings->startBitrate * 1000;
  bitrate_adjuster_.SetTargetBitrateBps(target_bitrate_bps_);

  RTC_LOG(LS_INFO) << "InitEncode " << target_bitrate_bps_ << "bit/sec";

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

  return WEBRTC_VIDEO_CODEC_OK;
}

int32_t MMALH264Encoder::Release() {
  std::lock_guard<std::mutex> lock(mtx_);
  MMALRelease();
  return WEBRTC_VIDEO_CODEC_OK;
}

int32_t MMALH264Encoder::MMALConfigure() {
  if (mmal_component_create(MMAL_COMPONENT_DEFAULT_VIDEO_ENCODER, &encoder_) !=
      MMAL_SUCCESS) {
    RTC_LOG(LS_ERROR) << "Failed to create mmal encoder";
    Release();
    return WEBRTC_VIDEO_CODEC_ERROR;
  }

  MMAL_COMPONENT_T* component_in;
  MMAL_ES_FORMAT_T* format_in;
  if (use_native_) {
    if (mmal_component_create("vc.ril.isp", &resizer_) != MMAL_SUCCESS) {
      RTC_LOG(LS_ERROR) << "Failed to create mmal resizer";
      Release();
      return WEBRTC_VIDEO_CODEC_ERROR;
    }

    if (input_video_type_ == webrtc::VideoType::kMJPEG) {
      if (mmal_component_create(MMAL_COMPONENT_DEFAULT_VIDEO_DECODER,
                                &decoder_) != MMAL_SUCCESS) {
        RTC_LOG(LS_ERROR) << "Failed to create mmal decoder";
        Release();
        return WEBRTC_VIDEO_CODEC_ERROR;
      }

      format_in = decoder_->input[0]->format;
      format_in->type = MMAL_ES_TYPE_VIDEO;
      format_in->encoding = MMAL_ENCODING_MJPEG;
      format_in->es->video.width = raw_width_;
      format_in->es->video.height = raw_height_;
      component_in = decoder_;

      MMAL_ES_FORMAT_T* decoder_out = decoder_->output[0]->format;
      mmal_format_copy(decoder_out, format_in);
      decoder_out->type = MMAL_ES_TYPE_VIDEO;
      decoder_out->encoding = MMAL_ENCODING_OPAQUE;
      decoder_out->encoding_variant = MMAL_ENCODING_I420;

      if (mmal_port_format_commit(decoder_->output[0]) != MMAL_SUCCESS) {
        RTC_LOG(LS_ERROR) << "Failed to commit decoder output port format";
        return WEBRTC_VIDEO_CODEC_ERROR;
      }
    } else {
      MMAL_FOURCC_T input_format;
      switch (input_video_type_) {
        case webrtc::VideoType::kI420:
          input_format = MMAL_ENCODING_I420;
          break;
        case webrtc::VideoType::kUYVY:
          input_format = MMAL_ENCODING_UYVY;
          break;
        case webrtc::VideoType::kYUY2:
          input_format = MMAL_ENCODING_YUYV;
          break;
        default:
          RTC_LOG(LS_ERROR) << "This video type is not supported in native "
                               "frame.  video type : "
                            << input_video_type_;
          Release();
          return WEBRTC_VIDEO_CODEC_ERROR;
      }
      format_in = resizer_->input[0]->format;
      format_in->type = MMAL_ES_TYPE_VIDEO;
      format_in->encoding = input_format;
      format_in->es->video.width = raw_width_;
      format_in->es->video.height = raw_height_;
      format_in->es->video.crop.x = 0;
      format_in->es->video.crop.y = 0;
      format_in->es->video.crop.width = raw_width_;
      format_in->es->video.crop.height = raw_height_;
      component_in = resizer_;
    }
  } else {
    format_in = encoder_->input[0]->format;
    format_in->type = MMAL_ES_TYPE_VIDEO;
    format_in->encoding = MMAL_ENCODING_I420;
    format_in->es->video.width = VCOS_ALIGN_UP(width_, 32);
    format_in->es->video.height = VCOS_ALIGN_UP(height_, 16);
    format_in->es->video.crop.x = 0;
    format_in->es->video.crop.y = 0;
    format_in->es->video.crop.width = width_;
    format_in->es->video.crop.height = height_;
    component_in = encoder_;
  }

  if (mmal_port_format_commit(component_in->input[0]) != MMAL_SUCCESS) {
    RTC_LOG(LS_ERROR) << "Failed to commit input port format";
    return WEBRTC_VIDEO_CODEC_ERROR;
  }

  /* Output port configure for H264 */
  MMAL_ES_FORMAT_T* format_out = encoder_->output[0]->format;
  mmal_format_copy(format_out, format_in);
  encoder_->output[0]->format->type = MMAL_ES_TYPE_VIDEO;
  encoder_->output[0]->format->encoding = MMAL_ENCODING_H264;
  encoder_->output[0]->format->es->video.frame_rate.num = 30;
  encoder_->output[0]->format->es->video.frame_rate.den = 1;
  encoder_->output[0]->format->bitrate =
      bitrate_adjuster_.GetAdjustedBitrateBps();

  if (mmal_port_format_commit(encoder_->output[0]) != MMAL_SUCCESS) {
    RTC_LOG(LS_ERROR) << "Failed to commit output port format";
    return WEBRTC_VIDEO_CODEC_ERROR;
  }

  if (mmal_port_parameter_set_boolean(component_in->input[0],
                                      MMAL_PARAMETER_ZERO_COPY,
                                      MMAL_FALSE) != MMAL_SUCCESS) {
    RTC_LOG(LS_ERROR) << "Failed to set component_in input zero copy";
    return WEBRTC_VIDEO_CODEC_ERROR;
  }

  if (mmal_port_parameter_set_boolean(component_in->output[0],
                                      MMAL_PARAMETER_ZERO_COPY,
                                      MMAL_TRUE) != MMAL_SUCCESS) {
    RTC_LOG(LS_ERROR) << "Failed to set component_in output zero copy";
    return WEBRTC_VIDEO_CODEC_ERROR;
  }

  if (mmal_port_parameter_set_boolean(encoder_->input[0],
                                      MMAL_PARAMETER_ZERO_COPY,
                                      MMAL_TRUE) != MMAL_SUCCESS) {
    RTC_LOG(LS_ERROR) << "Failed to set encoder input zero copy";
    return WEBRTC_VIDEO_CODEC_ERROR;
  }

  if (mmal_port_parameter_set_boolean(encoder_->output[0],
                                      MMAL_PARAMETER_ZERO_COPY,
                                      MMAL_TRUE) != MMAL_SUCCESS) {
    RTC_LOG(LS_ERROR) << "Failed to set encoder output zero copy";
    return WEBRTC_VIDEO_CODEC_ERROR;
  }

  MMAL_PARAMETER_VIDEO_PROFILE_T video_profile;
  video_profile.hdr.id = MMAL_PARAMETER_PROFILE;
  video_profile.hdr.size = sizeof(video_profile);

  video_profile.profile[0].profile = MMAL_VIDEO_PROFILE_H264_HIGH;
  video_profile.profile[0].level = MMAL_VIDEO_LEVEL_H264_42;

  if (mmal_port_parameter_set(encoder_->output[0], &video_profile.hdr) !=
      MMAL_SUCCESS) {
    RTC_LOG(LS_ERROR) << "Failed to set H264 profile";
    return WEBRTC_VIDEO_CODEC_ERROR;
  }

  if (mmal_port_parameter_set_uint32(encoder_->output[0],
                                     MMAL_PARAMETER_INTRAPERIOD,
                                     500) != MMAL_SUCCESS) {
    RTC_LOG(LS_ERROR) << "Failed to set intra period";
    return WEBRTC_VIDEO_CODEC_ERROR;
  }

  if (mmal_port_parameter_set_boolean(encoder_->output[0],
                                      MMAL_PARAMETER_VIDEO_ENCODE_INLINE_HEADER,
                                      MMAL_TRUE) != MMAL_SUCCESS) {
    RTC_LOG(LS_ERROR) << "Failed to set enable inline header";
    return WEBRTC_VIDEO_CODEC_ERROR;
  }

  if (mmal_port_parameter_set_boolean(encoder_->input[0],
                                      MMAL_PARAMETER_VIDEO_IMMUTABLE_INPUT,
                                      MMAL_TRUE) != MMAL_SUCCESS) {
    RTC_LOG(LS_ERROR) << "Failed to set enable immutable input";
    return WEBRTC_VIDEO_CODEC_ERROR;
  }

  component_in->input[0]->buffer_size =
      component_in->input[0]->buffer_size_recommended;
  if (component_in->input[0]->buffer_size <
      component_in->input[0]->buffer_size_min)
    component_in->input[0]->buffer_size =
        component_in->input[0]->buffer_size_min;
  if (input_video_type_ == webrtc::VideoType::kMJPEG)
    component_in->input[0]->buffer_size =
        component_in->input[0]->buffer_size_recommended * 8;
  component_in->input[0]->buffer_num = 1;
  component_in->input[0]->userdata = (MMAL_PORT_USERDATA_T*)this;

  if (mmal_port_enable(component_in->input[0], MMALInputCallbackFunction) !=
      MMAL_SUCCESS) {
    RTC_LOG(LS_ERROR) << "Failed to enable input port";
    return WEBRTC_VIDEO_CODEC_ERROR;
  }
  pool_in_ = mmal_port_pool_create(component_in->input[0],
                                   component_in->input[0]->buffer_num,
                                   component_in->input[0]->buffer_size);

  if (use_native_) {
    if (input_video_type_ == webrtc::VideoType::kMJPEG) {
      if (mmal_connection_create(
              &conn1_, decoder_->output[0], resizer_->input[0],
              MMAL_CONNECTION_FLAG_TUNNELLING) != MMAL_SUCCESS) {
        RTC_LOG(LS_ERROR) << "Failed to connect decoder to resizer";
        return WEBRTC_VIDEO_CODEC_ERROR;
      }
    }

    MMAL_ES_FORMAT_T* format_resize;
    format_resize = resizer_->output[0]->format;
    mmal_format_copy(format_resize, resizer_->input[0]->format);
    format_resize->encoding = MMAL_ENCODING_I420;
    format_resize->es->video.width = VCOS_ALIGN_UP(width_, 32);
    format_resize->es->video.height = VCOS_ALIGN_UP(height_, 16);
    format_resize->es->video.crop.x = 0;
    format_resize->es->video.crop.y = 0;
    format_resize->es->video.crop.width = width_;
    format_resize->es->video.crop.height = height_;
    if (mmal_port_format_commit(resizer_->output[0]) != MMAL_SUCCESS) {
      RTC_LOG(LS_ERROR) << "Failed to commit output port format";
      return WEBRTC_VIDEO_CODEC_ERROR;
    }

    if (mmal_connection_create(&conn2_, resizer_->output[0], encoder_->input[0],
                               MMAL_CONNECTION_FLAG_TUNNELLING) !=
        MMAL_SUCCESS) {
      RTC_LOG(LS_ERROR) << "Failed to connect resizer to encoder";
      return WEBRTC_VIDEO_CODEC_ERROR;
    }

    if (mmal_component_enable(resizer_) != MMAL_SUCCESS) {
      RTC_LOG(LS_ERROR) << "Failed to enable component";
      return WEBRTC_VIDEO_CODEC_ERROR;
    }

    if (input_video_type_ == webrtc::VideoType::kMJPEG) {
      if (mmal_component_enable(decoder_) != MMAL_SUCCESS) {
        RTC_LOG(LS_ERROR) << "Failed to enable component";
        return WEBRTC_VIDEO_CODEC_ERROR;
      }

      if (mmal_connection_enable(conn1_) != MMAL_SUCCESS) {
        RTC_LOG(LS_ERROR) << "Failed to enable connection decoder to resizer";
        return WEBRTC_VIDEO_CODEC_ERROR;
      }
    }

    if (mmal_connection_enable(conn2_) != MMAL_SUCCESS) {
      RTC_LOG(LS_ERROR) << "Failed to enable connection resizer to encoder";
      return WEBRTC_VIDEO_CODEC_ERROR;
    }
  }

  encoder_->output[0]->buffer_size =
      encoder_->output[0]->buffer_size_recommended * 4;
  if (encoder_->output[0]->buffer_size < encoder_->output[0]->buffer_size_min)
    encoder_->output[0]->buffer_size = encoder_->output[0]->buffer_size_min;
  encoder_->output[0]->buffer_num = 4;
  encoder_->output[0]->userdata = (MMAL_PORT_USERDATA_T*)this;

  encoded_image_buffer_.reset(new uint8_t[encoder_->output[0]->buffer_size]);

  if (mmal_port_enable(encoder_->output[0], MMALOutputCallbackFunction) !=
      MMAL_SUCCESS) {
    RTC_LOG(LS_ERROR) << "Failed to enable output port";
    return WEBRTC_VIDEO_CODEC_ERROR;
  }
  pool_out_ = mmal_port_pool_create(encoder_->output[0],
                                    encoder_->output[0]->buffer_num,
                                    encoder_->output[0]->buffer_size);

  if (mmal_component_enable(encoder_) != MMAL_SUCCESS) {
    RTC_LOG(LS_ERROR) << "Failed to enable component";
    return WEBRTC_VIDEO_CODEC_ERROR;
  }

  configured_width_ = width_;
  configured_height_ = height_;
  if (use_native_) {
    stride_width_ = VCOS_ALIGN_UP(raw_width_, 32);
    stride_height_ = VCOS_ALIGN_UP(raw_height_, 16);
  } else {
    stride_width_ = VCOS_ALIGN_UP(width_, 32);
    stride_height_ = VCOS_ALIGN_UP(height_, 16);
  }

  return WEBRTC_VIDEO_CODEC_OK;
}

void MMALH264Encoder::MMALRelease() {
  if (!encoder_)
    return;
  if (resizer_) {
    mmal_component_disable(resizer_);
    if (decoder_) {
      mmal_component_disable(decoder_);
      mmal_port_disable(decoder_->input[0]);
      mmal_port_pool_destroy(decoder_->input[0], pool_in_);
    } else {
      mmal_port_disable(resizer_->input[0]);
      mmal_port_pool_destroy(resizer_->input[0], pool_in_);
    }
  }
  if (encoder_) {
    mmal_component_disable(encoder_);
    if (!resizer_) {
      mmal_port_disable(encoder_->input[0]);
      mmal_port_pool_destroy(encoder_->input[0], pool_in_);
    }
    mmal_port_disable(encoder_->output[0]);
    mmal_port_pool_destroy(encoder_->output[0], pool_out_);
  }
  if (conn1_) {
    mmal_connection_destroy(conn1_);
    conn1_ = nullptr;
  }
  if (conn2_) {
    mmal_connection_destroy(conn2_);
    conn2_ = nullptr;
  }
  if (encoder_) {
    mmal_component_destroy(encoder_);
    encoder_ = nullptr;
  }
  if (resizer_) {
    mmal_component_destroy(resizer_);
    resizer_ = nullptr;
  }
  if (decoder_) {
    mmal_component_destroy(decoder_);
    decoder_ = nullptr;
  }
  while (!frame_params_.empty())
    frame_params_.pop();
  encoded_image_buffer_.reset();
  encoded_buffer_length_ = 0;
}

void MMALH264Encoder::MMALInputCallbackFunction(MMAL_PORT_T* port,
                                                MMAL_BUFFER_HEADER_T* buffer) {
  ((MMALH264Encoder*)port->userdata)->MMALInputCallback(port, buffer);
}

void MMALH264Encoder::MMALInputCallback(MMAL_PORT_T* port,
                                        MMAL_BUFFER_HEADER_T* buffer) {
  mmal_buffer_header_release(buffer);
}

void MMALH264Encoder::MMALOutputCallbackFunction(MMAL_PORT_T* port,
                                                 MMAL_BUFFER_HEADER_T* buffer) {
  ((MMALH264Encoder*)port->userdata)->MMALOutputCallback(port, buffer);
}

void MMALH264Encoder::MMALOutputCallback(MMAL_PORT_T* port,
                                         MMAL_BUFFER_HEADER_T* buffer) {
  if (buffer->length == 0) {
    mmal_buffer_header_release(buffer);
    return;
  }

  if (buffer->flags & MMAL_BUFFER_HEADER_FLAG_CONFIG) {
    memcpy(encoded_image_buffer_.get(), buffer->data, buffer->length);
    encoded_buffer_length_ = buffer->length;
    mmal_buffer_header_release(buffer);
    RTC_LOG(LS_INFO) << "MMAL_BUFFER_HEADER_FLAG_CONFIG";
    return;
  }

  RTC_LOG(LS_INFO) << "pts:" << buffer->pts << " flags:" << buffer->flags
                   << " planes:" << buffer->type->video.planes
                   << " length:" << buffer->length;

  std::unique_ptr<FrameParams> params;
  {
    rtc::CritScope lock(&frame_params_lock_);
    do {
      if (frame_params_.empty()) {
        RTC_LOG(LS_WARNING)
            << __FUNCTION__
            << "Frame parameter is not found. SkipFrame pts:" << buffer->pts;
        mmal_buffer_header_release(buffer);
        return;
      }
      params = std::move(frame_params_.front());
      frame_params_.pop();
    } while (params->timestamp < buffer->pts);
    if (params->timestamp != buffer->pts) {
      RTC_LOG(LS_WARNING) << __FUNCTION__
                          << "Frame parameter is not found. SkipFrame pts:"
                          << buffer->pts;
      mmal_buffer_header_release(buffer);
      return;
    }
  }

  encoded_image_._encodedWidth = params->width;
  encoded_image_._encodedHeight = params->height;
  encoded_image_.capture_time_ms_ = params->render_time_ms;
  encoded_image_.ntp_time_ms_ = params->ntp_time_ms;
  encoded_image_.SetTimestamp(buffer->pts);
  encoded_image_.rotation_ = params->rotation;
  encoded_image_.SetColorSpace(params->color_space);

  if (encoded_buffer_length_ == 0) {
    SendFrame(buffer->data, buffer->length);
  } else {
    memcpy(encoded_image_buffer_.get() + encoded_buffer_length_, buffer->data,
           buffer->length);
    encoded_buffer_length_ += buffer->length;
    SendFrame(encoded_image_buffer_.get(), encoded_buffer_length_);
    encoded_buffer_length_ = 0;
  }

  mmal_buffer_header_release(buffer);
}

int32_t MMALH264Encoder::RegisterEncodeCompleteCallback(
    webrtc::EncodedImageCallback* callback) {
  std::lock_guard<std::mutex> lock(mtx_);
  callback_ = callback;
  return WEBRTC_VIDEO_CODEC_OK;
}

void MMALH264Encoder::SetRates(const RateControlParameters& parameters) {
  if (encoder_ == nullptr)
    return;
  if (parameters.bitrate.get_sum_bps() <= 0 || parameters.framerate_fps <= 0)
    return;

  RTC_LOG(LS_INFO) << __FUNCTION__
                   << " bitrate:" << parameters.bitrate.get_sum_bps()
                   << " fps:" << parameters.framerate_fps;
  target_bitrate_bps_ = parameters.bitrate.get_sum_bps();
  bitrate_adjuster_.SetTargetBitrateBps(target_bitrate_bps_);
  target_framerate_fps_ = parameters.framerate_fps;
  return;
}

void MMALH264Encoder::SetBitrateBps(uint32_t bitrate_bps) {
  if (bitrate_bps < 300000 || configured_bitrate_bps_ == bitrate_bps) {
    return;
  }
  RTC_LOG(LS_INFO) << __FUNCTION__ << " " << bitrate_bps << " bit/sec";
  if (mmal_port_parameter_set_uint32(encoder_->output[0],
                                     MMAL_PARAMETER_VIDEO_BIT_RATE,
                                     bitrate_bps) != MMAL_SUCCESS) {
    RTC_LOG(LS_ERROR) << "Failed to set bitrate";
    return;
  }
  configured_bitrate_bps_ = bitrate_bps;
}

void MMALH264Encoder::SetFramerateFps(double framerate_fps) {
  if (configured_framerate_fps_ == framerate_fps) {
    return;
  }
  RTC_LOG(LS_ERROR) << __FUNCTION__ << " " << framerate_fps << " fps";
  MMAL_PARAMETER_FRAME_RATE_T frame_rate;
  frame_rate.hdr.id = MMAL_PARAMETER_VIDEO_FRAME_RATE;
  frame_rate.hdr.size = sizeof(frame_rate);
  frame_rate.frame_rate.num = framerate_fps;
  frame_rate.frame_rate.den = 1;
  if (mmal_port_parameter_set(encoder_->output[0], &frame_rate.hdr) !=
      MMAL_SUCCESS) {
    RTC_LOG(LS_ERROR) << "Failed to set H264 framerate";
    return;
  }
  configured_framerate_fps_ = framerate_fps;
}

webrtc::VideoEncoder::EncoderInfo MMALH264Encoder::GetEncoderInfo() const {
  EncoderInfo info;
  info.supports_native_handle = true;
  info.implementation_name = "MMAL H264";
  info.scaling_settings =
      VideoEncoder::ScalingSettings(kLowH264QpThreshold, kHighH264QpThreshold);
  info.is_hardware_accelerated = true;
  info.has_internal_source = false;
  return info;
}

int32_t MMALH264Encoder::Encode(
    const webrtc::VideoFrame& input_frame,
    const std::vector<webrtc::VideoFrameType>* frame_types) {
  std::lock_guard<std::mutex> lock(mtx_);
  if (!callback_) {
    RTC_LOG(LS_WARNING)
        << "InitEncode() has been called, but a callback function "
        << "has not been set with RegisterEncodeCompleteCallback()";
    return WEBRTC_VIDEO_CODEC_UNINITIALIZED;
  }

  rtc::scoped_refptr<webrtc::VideoFrameBuffer> frame_buffer =
      input_frame.video_frame_buffer();

  if (frame_buffer->width() != configured_width_ ||
      frame_buffer->height() != configured_height_) {
    RTC_LOG(LS_INFO) << "Encoder reinitialized from " << configured_width_
                     << "x" << configured_height_ << " to "
                     << frame_buffer->width() << "x" << frame_buffer->height();
    MMALRelease();
    if (frame_buffer->type() == webrtc::VideoFrameBuffer::Type::kNative) {
      NativeBuffer* native_buffer =
          dynamic_cast<NativeBuffer*>(frame_buffer.get());
      raw_width_ = native_buffer->raw_width();
      raw_height_ = native_buffer->raw_height();
      input_video_type_ = native_buffer->VideoType();
      use_native_ = true;
    } else {
      use_native_ = false;
    }
    if (MMALConfigure() != WEBRTC_VIDEO_CODEC_OK) {
      RTC_LOG(LS_ERROR) << "Failed to MMALConfigure";
      return WEBRTC_VIDEO_CODEC_ERROR;
    }
  }

  bool force_key_frame = false;
  if (frame_types != nullptr) {
    RTC_DCHECK_EQ(frame_types->size(), static_cast<size_t>(1));
    if ((*frame_types)[0] == webrtc::VideoFrameType::kEmptyFrame) {
      return WEBRTC_VIDEO_CODEC_OK;
    }
    force_key_frame =
        (*frame_types)[0] == webrtc::VideoFrameType::kVideoFrameKey;
  }

  if (force_key_frame) {
    if (mmal_port_parameter_set_boolean(encoder_->output[0],
                                        MMAL_PARAMETER_VIDEO_REQUEST_I_FRAME,
                                        MMAL_TRUE) != MMAL_SUCCESS) {
      RTC_LOG(LS_ERROR) << "Failed to request I frame";
    }
  }

  SetBitrateBps(bitrate_adjuster_.GetAdjustedBitrateBps());
  SetFramerateFps(target_framerate_fps_);
  {
    rtc::CritScope lock(&frame_params_lock_);
    frame_params_.push(absl::make_unique<FrameParams>(
        frame_buffer->width(), frame_buffer->height(),
        input_frame.render_time_ms(), input_frame.ntp_time_ms(),
        input_frame.timestamp(), input_frame.rotation(),
        input_frame.color_space()));
  }

  MMAL_BUFFER_HEADER_T* buffer;
  while ((buffer = mmal_queue_get(pool_out_->queue)) != nullptr) {
    if (mmal_port_send_buffer(encoder_->output[0], buffer) != MMAL_SUCCESS) {
      RTC_LOG(LS_ERROR) << "Failed to send output buffer";
      return WEBRTC_VIDEO_CODEC_ERROR;
    }
  }

  while ((buffer = mmal_queue_get(pool_in_->queue)) != nullptr) {
    buffer->pts = buffer->dts = input_frame.timestamp();
    buffer->offset = 0;
    buffer->flags = MMAL_BUFFER_HEADER_FLAG_FRAME;
    if (use_native_) {
      MMAL_COMPONENT_T* component_in;
      if (input_video_type_ == webrtc::VideoType::kMJPEG) {
        component_in = decoder_;
      } else {
        component_in = resizer_;
      }
      NativeBuffer* native_buffer =
          dynamic_cast<NativeBuffer*>(frame_buffer.get());
      buffer->length = buffer->alloc_size = native_buffer->length();
      buffer->data = (uint8_t *)native_buffer->Data();
      if (mmal_port_send_buffer(component_in->input[0], buffer) !=
          MMAL_SUCCESS) {
        RTC_LOG(LS_ERROR) << "Failed to send input native buffer";
        return WEBRTC_VIDEO_CODEC_ERROR;
      }
    } else {
      rtc::scoped_refptr<const webrtc::I420BufferInterface> i420_buffer =
          frame_buffer->ToI420();
      size_t offset = 0;
      for (size_t i = 0; i < i420_buffer->height(); i++) {
        memcpy(buffer->data + offset,
               (uint8_t*)i420_buffer->DataY() + (i420_buffer->StrideY() * i),
               i420_buffer->StrideY());
        offset += stride_width_;
      }
      offset = 0;
      size_t offset_y = stride_width_ * stride_height_;
      size_t width_uv = stride_width_ / 2;
      size_t offset_v = (stride_height_ / 2) * width_uv;
      for (size_t i = 0; i < ((i420_buffer->height() + 1) / 2); i++) {
        memcpy(buffer->data + offset_y + offset,
               (uint8_t*)i420_buffer->DataU() + (i420_buffer->StrideU() * i),
               width_uv);
        memcpy(buffer->data + offset_y + offset_v + offset,
               (uint8_t*)i420_buffer->DataV() + (i420_buffer->StrideV() * i),
               width_uv);
        offset += width_uv;
      }
      buffer->length = buffer->alloc_size = webrtc::CalcBufferSize(
          webrtc::VideoType::kI420, stride_width_, stride_height_);
      if (mmal_port_send_buffer(encoder_->input[0], buffer) != MMAL_SUCCESS) {
        RTC_LOG(LS_ERROR) << "Failed to send input i420 buffer";
        return WEBRTC_VIDEO_CODEC_ERROR;
      }
    }
  }

  return WEBRTC_VIDEO_CODEC_OK;
}

int32_t MMALH264Encoder::SendFrame(unsigned char* buffer, size_t size) {
  encoded_image_.set_buffer(buffer, size);
  encoded_image_.set_size(size);
  encoded_image_._frameType = webrtc::VideoFrameType::kVideoFrameDelta;

  uint8_t zero_count = 0;
  size_t nal_start_idx = 0;
  std::vector<nal_entry> nals;
  for (size_t i = 0; i < size; i++) {
    uint8_t data = buffer[i];
    if ((i != 0) && (i == nal_start_idx)) {
      if ((data & 0x1F) == 0x05) {
        encoded_image_._frameType = webrtc::VideoFrameType::kVideoFrameKey;
      }
    }
    if (data == 0x01 && zero_count == 3) {
      if (nal_start_idx != 0) {
        nals.push_back({nal_start_idx, i - nal_start_idx + 1 - 4});
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

  webrtc::RTPFragmentationHeader frag_header;
  frag_header.VerifyAndAllocateFragmentationHeader(nals.size());
  for (size_t i = 0; i < nals.size(); i++) {
    frag_header.fragmentationOffset[i] = nals[i].offset;
    frag_header.fragmentationLength[i] = nals[i].size;
  }

  webrtc::CodecSpecificInfo codec_specific;
  codec_specific.codecType = webrtc::kVideoCodecH264;
  codec_specific.codecSpecific.H264.packetization_mode =
      webrtc::H264PacketizationMode::NonInterleaved;

  h264_bitstream_parser_.ParseBitstream(buffer, size);
  h264_bitstream_parser_.GetLastSliceQp(&encoded_image_.qp_);
  RTC_LOG(LS_INFO) << __FUNCTION__ << " last slice qp:" << encoded_image_.qp_;

  webrtc::EncodedImageCallback::Result result =
      callback_->OnEncodedImage(encoded_image_, &codec_specific, &frag_header);
  if (result.error != webrtc::EncodedImageCallback::Result::OK) {
    RTC_LOG(LS_ERROR) << __FUNCTION__
                      << " OnEncodedImage failed error:" << result.error;
    return WEBRTC_VIDEO_CODEC_ERROR;
  }
  bitrate_adjuster_.Update(size);
  return WEBRTC_VIDEO_CODEC_OK;
}
