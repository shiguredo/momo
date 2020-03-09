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

#include "mmal_h264_decoder.h"

#include <unistd.h>

#include "common_video/libyuv/include/webrtc_libyuv.h"
#include "modules/video_coding/include/video_error_codes.h"
#include "rtc_base/checks.h"
#include "rtc_base/logging.h"
#include "rtc_base/time_utils.h"
#include "system_wrappers/include/metrics.h"
#include "third_party/libyuv/include/libyuv/convert.h"

MMALH264Decoder::MMALH264Decoder()
    : decoder_(nullptr),
      width_(0),
      height_(0),
      decode_complete_callback_(nullptr),
      buffer_pool_(false, 300 /* max_number_of_buffers*/) {
  bcm_host_init();
}

MMALH264Decoder::~MMALH264Decoder() {
  Release();
}

int32_t MMALH264Decoder::InitDecode(const webrtc::VideoCodec* codec_settings,
                                    int32_t number_of_cores) {
  RTC_LOG(LS_ERROR) << __FUNCTION__;
  if (MMALConfigure() != WEBRTC_VIDEO_CODEC_OK) {
    RTC_LOG(LS_ERROR) << __FUNCTION__ << "Failed to MMALConfigure";
    return WEBRTC_VIDEO_CODEC_ERROR;
  }
  return WEBRTC_VIDEO_CODEC_OK;
}

int32_t MMALH264Decoder::Decode(const webrtc::EncodedImage& input_image,
                                bool missing_frames,
                                int64_t render_time_ms) {
  if (decoder_ == nullptr) {
    return WEBRTC_VIDEO_CODEC_UNINITIALIZED;
  }
  if (decode_complete_callback_ == NULL) {
    return WEBRTC_VIDEO_CODEC_UNINITIALIZED;
  }
  if (input_image.data() == NULL && input_image.size() > 0) {
    return WEBRTC_VIDEO_CODEC_ERR_PARAMETER;
  }

  rtc::CritScope lock(&config_lock_);

  RTC_LOG(LS_INFO) << __FUNCTION__;

  MMAL_BUFFER_HEADER_T* buffer;
  if ((buffer = mmal_queue_get(pool_in_->queue)) != nullptr) {
    buffer->pts = buffer->dts = input_image.Timestamp();
    buffer->offset = 0;
    buffer->length = buffer->alloc_size = input_image.size();
    memcpy(buffer->data, input_image.data(), buffer->length);
    if (mmal_port_send_buffer(decoder_->input[0], buffer) != MMAL_SUCCESS) {
      RTC_LOG(LS_ERROR) << "Failed to send input buffer";
      return WEBRTC_VIDEO_CODEC_ERROR;
    }
    RTC_LOG(LS_INFO) << __FUNCTION__ << " timestamp:" << input_image.Timestamp()
                     << " size:" << input_image.size();
    return WEBRTC_VIDEO_CODEC_OK;
  } else {
    RTC_LOG(LS_ERROR) << "Failed to get buffer from input queue";
    return WEBRTC_VIDEO_CODEC_ERROR;
  }
}

int32_t MMALH264Decoder::RegisterDecodeCompleteCallback(
    webrtc::DecodedImageCallback* callback) {
  decode_complete_callback_ = callback;
  return WEBRTC_VIDEO_CODEC_OK;
}

int32_t MMALH264Decoder::Release() {
  MMALRelease();
  buffer_pool_.Release();
  return WEBRTC_VIDEO_CODEC_OK;
}

bool MMALH264Decoder::PrefersLateDecoding() const {
  return true;
}

const char* MMALH264Decoder::ImplementationName() const {
  return "MMAL H264";
}

void MMALH264Decoder::FillOutputBuffer() {
  MMAL_BUFFER_HEADER_T* buffer;
  while ((buffer = mmal_queue_get(pool_out_->queue)) != nullptr) {
    mmal_port_send_buffer(decoder_->output[0], buffer);
  }
}

void MMALH264Decoder::MMALInputCallbackFunction(MMAL_PORT_T* port,
                                                MMAL_BUFFER_HEADER_T* buffer) {
  mmal_buffer_header_release(buffer);
}

void MMALH264Decoder::MMALOutputCallbackFunction(MMAL_PORT_T* port,
                                                 MMAL_BUFFER_HEADER_T* buffer) {
  RTC_LOG(LS_INFO) << __FUNCTION__;
  ((MMALH264Decoder*)port->userdata)->MMALOutputCallback(port, buffer);
}

void MMALH264Decoder::MMALOutputCallback(MMAL_PORT_T* port,
                                         MMAL_BUFFER_HEADER_T* buffer) {
  RTC_LOG(LS_INFO) << __FUNCTION__ << " cmd:" << buffer->cmd
                   << " length:" << buffer->length;

  if (buffer->cmd == MMAL_EVENT_FORMAT_CHANGED) {
    rtc::CritScope lock(&config_lock_);
    MMAL_EVENT_FORMAT_CHANGED_T* event = mmal_event_format_changed_get(buffer);

    width_ = event->format->es->video.width;
    height_ = event->format->es->video.height;

    RTC_LOG(LS_INFO) << __FUNCTION__ << " width:" << width_
                     << " height:" << height_;

    MMAL_PORT_T* port_out = decoder_->output[0];

    mmal_format_full_copy(port_out->format, event->format);
    port_out->format->encoding = MMAL_ENCODING_I420;
    if (mmal_port_format_commit(port_out) != MMAL_SUCCESS) {
      RTC_LOG(LS_ERROR) << "Failed to commit output port format";
      return;
    }

    port_out->buffer_size = port_out->buffer_size_recommended;
    port_out->buffer_num = 3;
    port_out->userdata = (MMAL_PORT_USERDATA_T*)this;
  } else {
    SendFrame(buffer);
  }

  mmal_buffer_header_release(buffer);
  FillOutputBuffer();
}

void MMALH264Decoder::SendFrame(MMAL_BUFFER_HEADER_T* buffer) {
  rtc::scoped_refptr<webrtc::I420Buffer> i420_buffer =
      buffer_pool_.CreateBuffer(width_, height_);
  if (!i420_buffer.get()) {
    return;
  }

  memcpy(i420_buffer->MutableDataY(), buffer->data, buffer->length);

  webrtc::VideoFrame decoded_image = webrtc::VideoFrame::Builder()
                                         .set_video_frame_buffer(i420_buffer)
                                         .set_timestamp_rtp(buffer->pts)
                                         .build();
  decode_complete_callback_->Decoded(decoded_image, absl::nullopt,
                                     absl::nullopt);
}

int32_t MMALH264Decoder::MMALConfigure() {
  if (mmal_component_create(MMAL_COMPONENT_DEFAULT_VIDEO_DECODER, &decoder_) !=
      MMAL_SUCCESS) {
    RTC_LOG(LS_ERROR) << "Failed to create mmal decoder";
    Release();
    return WEBRTC_VIDEO_CODEC_ERROR;
  }

  MMAL_PORT_T* port_in = decoder_->input[0];
  port_in->format->type = MMAL_ES_TYPE_VIDEO;
  port_in->format->encoding = MMAL_ENCODING_H264;
  port_in->format->es->video.width = 0;
  port_in->format->es->video.height = 0;
  port_in->format->es->video.frame_rate.num = 0;
  port_in->format->es->video.frame_rate.den = 1;
  port_in->format->es->video.par.num = 1;
  port_in->format->es->video.par.den = 1;

  if (mmal_port_format_commit(port_in) != MMAL_SUCCESS) {
    RTC_LOG(LS_ERROR) << "Failed to commit input port format";
    return -1;
  }

  MMAL_PORT_T* port_out = decoder_->output[0];
  mmal_format_copy(port_out->format, port_in->format);
  port_out->format->encoding = MMAL_ENCODING_I420;

  if (mmal_port_format_commit(port_out) != MMAL_SUCCESS) {
    RTC_LOG(LS_ERROR) << "Failed to commit output port format";
    return -1;
  }

  port_in->buffer_num = 3;
  port_in->buffer_size = 256 << 10;
  port_out->buffer_num = 3;
  port_out->buffer_size = 256;

  port_in->userdata = (MMAL_PORT_USERDATA_T*)this;
  port_out->userdata = (MMAL_PORT_USERDATA_T*)this;

  if (mmal_port_parameter_set_boolean(port_in, MMAL_PARAMETER_ZERO_COPY,
                                      MMAL_TRUE) != MMAL_SUCCESS) {
    RTC_LOG(LS_ERROR) << "Failed to set resizer input zero copy";
    return -1;
  }

  if (mmal_port_parameter_set_boolean(port_out, MMAL_PARAMETER_ZERO_COPY,
                                      MMAL_TRUE) != MMAL_SUCCESS) {
    RTC_LOG(LS_ERROR) << "Failed to set resizer output zero copy";
    return -1;
  }

  if (mmal_port_enable(port_in, MMALInputCallbackFunction) != MMAL_SUCCESS) {
    RTC_LOG(LS_ERROR) << "Failed to enable input port";
    return -1;
  }

  if (mmal_port_enable(port_out, MMALOutputCallbackFunction) != MMAL_SUCCESS) {
    RTC_LOG(LS_ERROR) << "Failed to enable output port";
    return WEBRTC_VIDEO_CODEC_ERROR;
  }

  pool_in_ =
      mmal_port_pool_create(port_in, port_in->buffer_num, port_in->buffer_size);
  pool_out_ = mmal_port_pool_create(
      port_out, port_out->buffer_num,
      webrtc::CalcBufferSize(webrtc::VideoType::kI420, 1920, 1088));

  if (mmal_component_enable(decoder_) != MMAL_SUCCESS) {
    RTC_LOG(LS_ERROR) << "Failed to enable component";
    return -1;
  }

  FillOutputBuffer();

  return WEBRTC_VIDEO_CODEC_OK;
}

void MMALH264Decoder::MMALRelease() {
  if (decoder_) {
    mmal_component_disable(decoder_);
    mmal_port_disable(decoder_->input[0]);
    mmal_port_pool_destroy(decoder_->input[0], pool_in_);
    mmal_port_disable(decoder_->output[0]);
    mmal_port_pool_destroy(decoder_->output[0], pool_out_);
    mmal_component_destroy(decoder_);
    decoder_ = nullptr;
  }
}