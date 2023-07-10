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

#include "v4l2_h264_encoder.h"

#include <iostream>
#include <limits>
#include <string>

#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

// WebRTC
#include <common_video/libyuv/include/webrtc_libyuv.h>
#include <rtc_base/checks.h>
#include <rtc_base/logging.h>
#include <rtc_base/time_utils.h>
#include <system_wrappers/include/metrics.h>
#include <third_party/libyuv/include/libyuv/convert.h>
#include <third_party/libyuv/include/libyuv/convert_from.h>
#include <third_party/libyuv/include/libyuv/video_common.h>

namespace {

const int kLowH264QpThreshold = 34;
const int kHighH264QpThreshold = 40;

}  // namespace

V4L2H264Encoder::V4L2H264Encoder(const cricket::VideoCodec& codec)
    : configured_width_(0),
      configured_height_(0),
      callback_(nullptr),
      bitrate_adjuster_(.5, .95),
      target_framerate_fps_(30),
      configured_framerate_fps_(30) {}

V4L2H264Encoder::~V4L2H264Encoder() {}

int32_t V4L2H264Encoder::InitEncode(
    const webrtc::VideoCodec* codec_settings,
    const webrtc::VideoEncoder::Settings& settings) {
  RTC_DCHECK(codec_settings);
  RTC_DCHECK_EQ(codec_settings->codecType, webrtc::kVideoCodecH264);

  target_bitrate_bps_ = codec_settings->startBitrate * 1000;
  bitrate_adjuster_.SetTargetBitrateBps(target_bitrate_bps_);

  RTC_LOG(LS_INFO) << __FUNCTION__ << "  width: " << codec_settings->width
                   << "  height: " << codec_settings->height
                   << "  bitrate: " << target_bitrate_bps_ << "bit/sec";

  encoded_image_.timing_.flags =
      webrtc::VideoSendTiming::TimingFrameFlags::kInvalid;
  encoded_image_.content_type_ =
      (codec_settings->mode == webrtc::VideoCodecMode::kScreensharing)
          ? webrtc::VideoContentType::SCREENSHARE
          : webrtc::VideoContentType::UNSPECIFIED;

  return Configure(webrtc::VideoFrameBuffer::Type::kI420,
                   webrtc::VideoType::kI420, codec_settings->width,
                   codec_settings->height, codec_settings->width,
                   codec_settings->width, codec_settings->height);
}

int32_t V4L2H264Encoder::Configure(webrtc::VideoFrameBuffer::Type type,
                                   webrtc::VideoType video_type,
                                   int32_t raw_width,
                                   int32_t raw_height,
                                   int32_t raw_stride,
                                   int32_t width,
                                   int32_t height) {
  int memory = type == webrtc::VideoFrameBuffer::Type::kNative
                   ? V4L2_MEMORY_DMABUF
                   : V4L2_MEMORY_MMAP;
  if (video_type == webrtc::VideoType::kMJPEG) {
    jpeg_decoder_ = V4L2Decoder::Create(V4L2_PIX_FMT_MJPEG, true, raw_width,
                                        raw_height, raw_stride);
    if (jpeg_decoder_ == nullptr) {
      RTC_LOG(LS_ERROR) << "Failed to MJPEG decoder";
      return WEBRTC_VIDEO_CODEC_ERROR;
    }
  }
  if (memory == V4L2_MEMORY_DMABUF) {
    scaler_ = V4L2Scaler::Create(V4L2_MEMORY_DMABUF, raw_width, raw_height,
                                 raw_stride, true, width, height, width);
    if (scaler_ == nullptr) {
      RTC_LOG(LS_ERROR) << "Failed to create scaler";
      return WEBRTC_VIDEO_CODEC_ERROR;
    }
  }
  h264_converter_ = V4L2H264Converter::Create(memory, width, height, width);
  if (h264_converter_ == nullptr) {
    return WEBRTC_VIDEO_CODEC_ERROR;
  }
  configured_type_ = type;
  configured_width_ = width;
  configured_height_ = height;

  return WEBRTC_VIDEO_CODEC_OK;
}

int32_t V4L2H264Encoder::Release() {
  h264_converter_.reset();
  return WEBRTC_VIDEO_CODEC_OK;
}

int32_t V4L2H264Encoder::RegisterEncodeCompleteCallback(
    webrtc::EncodedImageCallback* callback) {
  std::lock_guard<std::mutex> lock(callback_mutex_);
  callback_ = callback;
  return WEBRTC_VIDEO_CODEC_OK;
}

void V4L2H264Encoder::SetRates(const RateControlParameters& parameters) {
  if (h264_converter_ == nullptr)
    return;
  if (parameters.bitrate.get_sum_bps() <= 0 || parameters.framerate_fps <= 0)
    return;

  RTC_LOG(LS_INFO) << __FUNCTION__
                   << "  bitrate:" << parameters.bitrate.get_sum_bps()
                   << "  fps:" << parameters.framerate_fps;
  target_bitrate_bps_ = parameters.bitrate.get_sum_bps();
  bitrate_adjuster_.SetTargetBitrateBps(target_bitrate_bps_);
  target_framerate_fps_ = parameters.framerate_fps;
  return;
}

void V4L2H264Encoder::SetBitrateBps(uint32_t bitrate_bps) {
  if (h264_converter_ == nullptr)
    return;
  if (bitrate_bps < 300000 || configured_bitrate_bps_ == bitrate_bps) {
    return;
  }
  RTC_LOG(LS_INFO) << __FUNCTION__ << "  bitrate: " << bitrate_bps
                   << " bit/sec";
  v4l2_control ctrl = {};
  ctrl.id = V4L2_CID_MPEG_VIDEO_BITRATE;
  ctrl.value = bitrate_bps;
  if (ioctl(h264_converter_->fd(), VIDIOC_S_CTRL, &ctrl) < 0) {
    RTC_LOG(LS_ERROR) << __FUNCTION__ << "  Failed to set bitrate";
    return;
  }
  configured_bitrate_bps_ = bitrate_bps;
}

void V4L2H264Encoder::SetFramerateFps(double framerate_fps) {
  if (h264_converter_ == nullptr)
    return;
  if (configured_framerate_fps_ == framerate_fps) {
    return;
  }
  RTC_LOG(LS_INFO) << __FUNCTION__ << "  fps: " << framerate_fps;
  v4l2_streamparm stream = {};
  stream.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
  stream.parm.output.timeperframe.numerator = 1;
  stream.parm.output.timeperframe.denominator = framerate_fps;
  if (ioctl(h264_converter_->fd(), VIDIOC_S_PARM, &stream) < 0) {
    RTC_LOG(LS_ERROR) << __FUNCTION__ << "  Failed to set framerate";
    return;
  }
  configured_framerate_fps_ = framerate_fps;
}

webrtc::VideoEncoder::EncoderInfo V4L2H264Encoder::GetEncoderInfo() const {
  EncoderInfo info;
  info.supports_native_handle = true;
  info.implementation_name = "V4L2 H264";
  info.scaling_settings =
      VideoEncoder::ScalingSettings(kLowH264QpThreshold, kHighH264QpThreshold);
  return info;
}

int32_t V4L2H264Encoder::Encode(
    const webrtc::VideoFrame& input_frame,
    const std::vector<webrtc::VideoFrameType>* frame_types) {
  {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    if (!callback_) {
      RTC_LOG(LS_WARNING)
          << "InitEncode() has been called, but a callback function "
          << "has not been set with RegisterEncodeCompleteCallback()";
      return WEBRTC_VIDEO_CODEC_UNINITIALIZED;
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

  rtc::scoped_refptr<webrtc::VideoFrameBuffer> frame_buffer =
      input_frame.video_frame_buffer();

  RTC_LOG(LS_INFO) << "V4L2H264Encoder::Encode: type="
                   << (int)frame_buffer->type();
  if (frame_buffer->type() != configured_type_ ||
      frame_buffer->width() != configured_width_ ||
      frame_buffer->height() != configured_height_) {
    RTC_LOG(LS_INFO) << "Encoder reinitialized from " << configured_width_
                     << "x" << configured_height_ << " to "
                     << frame_buffer->width() << "x" << frame_buffer->height();
    Release();
    webrtc::VideoType video_type = webrtc::VideoType::kI420;
    int stride = frame_buffer->width();
    int raw_width = frame_buffer->width();
    int raw_height = frame_buffer->height();
    if (frame_buffer->type() == webrtc::VideoFrameBuffer::Type::kNative) {
      auto native_buffer = static_cast<V4L2NativeBuffer*>(frame_buffer.get());
      video_type = native_buffer->video_type();
      stride = native_buffer->stride();
      raw_width = native_buffer->raw_width();
      raw_height = native_buffer->raw_height();
    }
    if (Configure(frame_buffer->type(), video_type, raw_width, raw_height,
                  stride, frame_buffer->width(),
                  frame_buffer->height()) != WEBRTC_VIDEO_CODEC_OK) {
      RTC_LOG(LS_ERROR) << __FUNCTION__ << "  Failed to Configure";
      return WEBRTC_VIDEO_CODEC_ERROR;
    }
  }

  SetBitrateBps(bitrate_adjuster_.GetAdjustedBitrateBps());
  SetFramerateFps(target_framerate_fps_);

  if (scaler_ == nullptr) {
    h264_converter_->Encode(
        frame_buffer, input_frame.timestamp_us(), force_key_frame,
        [this, input_frame](uint8_t* buffer, int size, int64_t timestamp_us,
                            bool is_key_frame) {
          SendFrame(input_frame, buffer, size, timestamp_us, is_key_frame);
        });
  } else {
    if (jpeg_decoder_ == nullptr) {
      scaler_->Scale(
          frame_buffer, input_frame.timestamp_us(),
          [this, force_key_frame, input_frame](
              rtc::scoped_refptr<webrtc::VideoFrameBuffer> buffer,
              int64_t timestamp_us) {
            h264_converter_->Encode(
                buffer, timestamp_us, force_key_frame,
                [this, input_frame](uint8_t* buffer, int size,
                                    int64_t timestamp_us, bool is_key_frame) {
                  SendFrame(input_frame, buffer, size, timestamp_us,
                            is_key_frame);
                });
          });
    } else {
      auto native_buffer = static_cast<V4L2NativeBuffer*>(frame_buffer.get());
      jpeg_decoder_->Decode(
          native_buffer->data().get(), native_buffer->size(),
          input_frame.timestamp(),
          [this, force_key_frame, input_frame](
              rtc::scoped_refptr<webrtc::VideoFrameBuffer> buffer,
              int64_t timestamp_rtp) {
            RTC_LOG(LS_INFO) << "Decoded JPEG frame: type=" << buffer->type()
                             << " width=" << buffer->width()
                             << " height=" << buffer->height();
            scaler_->Scale(
                buffer, input_frame.timestamp_us(),
                [this, force_key_frame, input_frame](
                    rtc::scoped_refptr<webrtc::VideoFrameBuffer> buffer,
                    int64_t timestamp_us) {
                  h264_converter_->Encode(
                      buffer, timestamp_us, force_key_frame,
                      [this, input_frame](uint8_t* buffer, int size,
                                          int64_t timestamp_us,
                                          bool is_key_frame) {
                        SendFrame(input_frame, buffer, size, timestamp_us,
                                  is_key_frame);
                      });
                });
          });
    }
  }

  return WEBRTC_VIDEO_CODEC_OK;
}

int32_t V4L2H264Encoder::SendFrame(const webrtc::VideoFrame& frame,
                                   unsigned char* buffer,
                                   size_t size,
                                   int64_t timestamp_us,
                                   bool is_key_frame) {
  if (frame.timestamp_us() != timestamp_us) {
    RTC_LOG(LS_ERROR) << __FUNCTION__
                      << "  Frame parameter is not found. SkipFrame"
                      << "  timestamp_us: " << timestamp_us;
    return WEBRTC_VIDEO_CODEC_ERROR;
  }

  auto encoded_image_buffer = webrtc::EncodedImageBuffer::Create(buffer, size);
  encoded_image_.SetEncodedData(encoded_image_buffer);

  encoded_image_._encodedWidth = frame.width();
  encoded_image_._encodedHeight = frame.height();
  encoded_image_.capture_time_ms_ = frame.render_time_ms();
  encoded_image_.ntp_time_ms_ = frame.ntp_time_ms();
  encoded_image_.SetTimestamp(frame.timestamp());
  encoded_image_.rotation_ = frame.rotation();
  encoded_image_.SetColorSpace(frame.color_space());
  encoded_image_._frameType = is_key_frame
                                  ? webrtc::VideoFrameType::kVideoFrameKey
                                  : webrtc::VideoFrameType::kVideoFrameDelta;

  h264_bitstream_parser_.ParseBitstream(encoded_image_);
  encoded_image_.qp_ = h264_bitstream_parser_.GetLastSliceQp().value_or(-1);
  RTC_LOG(LS_INFO) << __FUNCTION__ << "  qp:" << encoded_image_.qp_;

  webrtc::CodecSpecificInfo codec_specific;
  codec_specific.codecType = webrtc::kVideoCodecH264;
  codec_specific.codecSpecific.H264.packetization_mode =
      webrtc::H264PacketizationMode::NonInterleaved;

  webrtc::EncodedImageCallback::Result result =
      callback_->OnEncodedImage(encoded_image_, &codec_specific);
  if (result.error != webrtc::EncodedImageCallback::Result::OK) {
    RTC_LOG(LS_ERROR) << __FUNCTION__ << "  OnEncodedImage failed"
                      << "  error:" << result.error;
    return WEBRTC_VIDEO_CODEC_ERROR;
  }
  bitrate_adjuster_.Update(size);
  return WEBRTC_VIDEO_CODEC_OK;
}
