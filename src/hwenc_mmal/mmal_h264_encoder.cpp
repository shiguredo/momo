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

#include <chrono>
#include <limits>
#include <string>

#include "third_party/libyuv/include/libyuv/convert.h"
#include "third_party/libyuv/include/libyuv/convert_from.h"
#include "third_party/libyuv/include/libyuv/video_common.h"

#include "rtc_base/checks.h"
#include "rtc_base/logging.h"
#include "common_video/libyuv/include/webrtc_libyuv.h"
#include "system_wrappers/include/metrics.h"

#define H264HWENC_HEADER_DEBUG 0
#define ROUND_UP_4(num) (((num) + 3) & ~3)

namespace
{
struct nal_entry
{
  size_t offset;
  size_t size;
};

const uint32_t kFramerate = 30;
const int kLowH264QpThreshold = 24;
const int kHighH264QpThreshold = 37;

int I420DataSize(const webrtc::I420BufferInterface &frame_buffer)
{
  return frame_buffer.StrideY() * frame_buffer.height() + (frame_buffer.StrideU() + frame_buffer.StrideV()) * ((frame_buffer.height() + 1) / 2);
}

} // namespace

MMALH264Encoder::MMALH264Encoder(const cricket::VideoCodec &codec)
    : callback_(nullptr),
      encoder_(nullptr),
      queue_(nullptr),
      pool_out_(nullptr),
      bitrate_adjuster_(.5, .95),
      configured_width_(0),
      configured_height_(0),
      buffer_size_(0)
{
}

MMALH264Encoder::~MMALH264Encoder()
{
  Release();
}

int32_t MMALH264Encoder::InitEncode(const webrtc::VideoCodec *codec_settings,
                                  int32_t number_of_cores,
                                  size_t max_payload_size)
{
  RTC_DCHECK(codec_settings);
  RTC_DCHECK_EQ(codec_settings->codecType, webrtc::kVideoCodecH264);

  int32_t release_ret = Release();
  if (release_ret != WEBRTC_VIDEO_CODEC_OK)
  {
    return release_ret;
  }

  bcm_host_init();

  if (vcos_semaphore_create(&semaphore_, "encoder sem", 0) != VCOS_SUCCESS)
  {
    RTC_LOG(LS_ERROR) << "Failed to create mmal semaphore";
    return WEBRTC_VIDEO_CODEC_ERROR;
  }

  if (mmal_component_create(MMAL_COMPONENT_DEFAULT_VIDEO_ENCODER, &encoder_) != MMAL_SUCCESS)
  {
    RTC_LOG(LS_ERROR) << "Failed to create mmal component";
    Release();
    return WEBRTC_VIDEO_CODEC_ERROR;
  }

  width_ = codec_settings->width;
  height_ = codec_settings->height;
  target_bitrate_bps_ = codec_settings->startBitrate * 1000;
  bitrate_adjuster_.SetTargetBitrateBps(target_bitrate_bps_);

  RTC_LOG(LS_ERROR) << "InitEncode " << (uint32_t)(codec_settings->maxFramerate) << "fps "
                    << (uint32_t)(target_bitrate_bps_) << "bit/sec";

  // Initialize encoded image. Default buffer size: size of unencoded data.
  buffer_size_ = CalcBufferSize(
      webrtc::VideoType::kI420, codec_settings->width, codec_settings->height);
  encoded_image_buffer_.reset(new uint8_t[buffer_size_]);
  encoded_image_.set_buffer(encoded_image_buffer_.get(), buffer_size_);
  encoded_image_._completeFrame = true;
  encoded_image_._encodedWidth = 0;
  encoded_image_._encodedHeight = 0;
  encoded_image_.set_size(0);
  encoded_image_.content_type_ = webrtc::VideoContentType::UNSPECIFIED;
  encoded_image_.timing_.flags = webrtc::VideoSendTiming::TimingFrameFlags::kInvalid;
  encoded_image_.content_type_ = (codec_settings->mode == webrtc::VideoCodecMode::kScreensharing)
                                     ? webrtc::VideoContentType::SCREENSHARE
                                     : webrtc::VideoContentType::UNSPECIFIED;

  return WEBRTC_VIDEO_CODEC_OK;
}

int32_t MMALH264Encoder::Release()
{
  if (encoder_)
  {
    mmal_component_destroy(encoder_);
    encoder_ = nullptr;
  }
  vcos_semaphore_delete(&semaphore_);
  encoded_image_buffer_.reset();
  buffer_size_ = 0;
  return WEBRTC_VIDEO_CODEC_OK;
}

int32_t MMALH264Encoder::MMALConfigure()
{
  int32_t stride_width = VCOS_ALIGN_UP(width_, 32);
  int32_t stride_height = VCOS_ALIGN_UP(height_, 16);

  /* Input port configure for RAW data */
  MMAL_ES_FORMAT_T *format_in = encoder_->input[0]->format;
  format_in->type = MMAL_ES_TYPE_VIDEO;
  format_in->encoding = MMAL_ENCODING_I420;
  format_in->es->video.width = stride_width;
  format_in->es->video.height = stride_height;
  format_in->es->video.crop.x = 0;
  format_in->es->video.crop.y = 0;
  format_in->es->video.crop.width = width_;
  format_in->es->video.crop.height = height_;
  format_in->es->video.frame_rate.num = 30;
  format_in->es->video.frame_rate.den = 1;
  format_in->es->video.par.num = 1;
  format_in->es->video.par.den = 1;

  if (mmal_port_format_commit(encoder_->input[0]) != MMAL_SUCCESS)
  {
    RTC_LOG(LS_ERROR) << "Failed to commit input port format";
    return WEBRTC_VIDEO_CODEC_ERROR;
  }

  /* Output port configure for H264 */
  MMAL_ES_FORMAT_T *format_out = encoder_->output[0]->format;
  mmal_format_copy(format_out, format_in);
  encoder_->output[0]->format->type = MMAL_ES_TYPE_VIDEO;
  encoder_->output[0]->format->encoding = MMAL_ENCODING_H264;
  encoder_->output[0]->format->bitrate = target_bitrate_bps_;

  if (mmal_port_format_commit(encoder_->output[0]) != MMAL_SUCCESS)
  {
    RTC_LOG(LS_ERROR) << "Failed to commit output port format";
    return WEBRTC_VIDEO_CODEC_ERROR;
  }

  if (mmal_port_parameter_set_boolean(encoder_->input[0], MMAL_PARAMETER_ZERO_COPY, MMAL_TRUE) != MMAL_SUCCESS)
  {
    RTC_LOG(LS_ERROR) << "Failed to set input zero copy";
    return WEBRTC_VIDEO_CODEC_ERROR;
  }

  /*if (mmal_port_parameter_set_boolean(encoder_->output[0], MMAL_PARAMETER_ZERO_COPY, MMAL_TRUE) != MMAL_SUCCESS)
  {
    RTC_LOG(LS_ERROR) << "Failed to set output zero copy";
    return WEBRTC_VIDEO_CODEC_ERROR;
  }*/

  MMAL_PARAMETER_VIDEO_PROFILE_T video_profile;
  video_profile.hdr.id = MMAL_PARAMETER_PROFILE;
  video_profile.hdr.size = sizeof(video_profile);

  video_profile.profile[0].profile = MMAL_VIDEO_PROFILE_H264_BASELINE;
  video_profile.profile[0].level = MMAL_VIDEO_LEVEL_H264_4;

  if (mmal_port_parameter_set(encoder_->output[0], &video_profile.hdr) != MMAL_SUCCESS)
  {
    RTC_LOG(LS_ERROR) << "Failed to set H264 profile";
    return WEBRTC_VIDEO_CODEC_ERROR;
  }

  if (mmal_port_parameter_set_boolean(encoder_->output[0], MMAL_PARAMETER_VIDEO_ENCODE_INLINE_HEADER, MMAL_TRUE) != MMAL_SUCCESS)
  {
    RTC_LOG(LS_ERROR) << "Failed to set enable inline header";
    return WEBRTC_VIDEO_CODEC_ERROR;
  }

  if (mmal_port_parameter_set_uint32(encoder_->output[0], MMAL_PARAMETER_RATECONTROL, MMAL_VIDEO_RATECONTROL_CONSTANT_SKIP_FRAMES) != MMAL_SUCCESS)
  {
    RTC_LOG(LS_ERROR) << "Failed to set rate control";
    return WEBRTC_VIDEO_CODEC_ERROR;
  }

  queue_ = mmal_queue_create();

  encoder_->input[0]->buffer_size = encoder_->input[0]->buffer_size_recommended;
  if (encoder_->input[0]->buffer_size < encoder_->input[0]->buffer_size_min)
    encoder_->input[0]->buffer_size = encoder_->input[0]->buffer_size_min;
  encoder_->input[0]->buffer_num = 1;
  encoder_->input[0]->userdata = (MMAL_PORT_USERDATA_T *)this;

  //RTC_LOG(LS_ERROR) << "buffer_size_recommended:" << encoder_->input[0]->buffer_size_recommended;

  if (mmal_port_enable(encoder_->input[0], MMALInputCallbackFunction) != MMAL_SUCCESS)
  {
    RTC_LOG(LS_ERROR) << "Failed to enable input port";
    return WEBRTC_VIDEO_CODEC_ERROR;
  }
  pool_in_ = mmal_port_pool_create(encoder_->input[0], encoder_->input[0]->buffer_num, encoder_->input[0]->buffer_size);

  encoder_->output[0]->buffer_size = encoder_->output[0]->buffer_size_recommended;
  if (encoder_->output[0]->buffer_size < encoder_->output[0]->buffer_size_min)
    encoder_->output[0]->buffer_size = encoder_->output[0]->buffer_size_min;
  encoder_->output[0]->buffer_num = 1;
  encoder_->output[0]->userdata = (MMAL_PORT_USERDATA_T *)this;

  if (mmal_port_enable(encoder_->output[0], MMALOutputCallbackFunction) != MMAL_SUCCESS)
  {
    RTC_LOG(LS_ERROR) << "Failed to enable output port";
    return WEBRTC_VIDEO_CODEC_ERROR;
  }
  pool_out_ = mmal_port_pool_create(encoder_->output[0], encoder_->output[0]->buffer_num, encoder_->output[0]->buffer_size);

  if (mmal_component_enable(encoder_) != MMAL_SUCCESS)
  {
    RTC_LOG(LS_ERROR) << "Failed to enable component";
    return WEBRTC_VIDEO_CODEC_ERROR;
  }

  configured_width_ = width_;
  configured_height_ = height_;
  stride_width_ = stride_width;
  stride_height_ = stride_height;

  return WEBRTC_VIDEO_CODEC_OK;
}

void MMALH264Encoder::MMALRelease()
{
  if (encoder_ == nullptr)
    return;
  if (encoder_->input[0]->is_enabled)
  {
    if (mmal_port_disable(encoder_->input[0]) != MMAL_SUCCESS)
    {
      RTC_LOG(LS_ERROR) << "Failed to disable input port";
    }
  }
  if (encoder_->output[0]->is_enabled)
  {
    if (mmal_port_disable(encoder_->output[0]) != MMAL_SUCCESS)
    {
      RTC_LOG(LS_ERROR) << "Failed to disable output port";
    }
  }
  if (queue_ != nullptr)
  {
    MMAL_BUFFER_HEADER_T *buffer;
    while ((buffer = mmal_queue_get(queue_)) != nullptr)
    {
      mmal_buffer_header_release(buffer);
    }
    if (pool_out_ != nullptr)
    {
      if (!vcos_verify(mmal_queue_length(pool_out_->queue) == pool_out_->headers_num))
      {
        RTC_LOG(LS_ERROR) << "Failed to release all output buffers";
      }
      pool_out_ = nullptr;
    }
    queue_ = nullptr;
  }
}

void MMALH264Encoder::MMALInputCallbackFunction(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer)
{
  ((MMALH264Encoder *)port->userdata)->MMALInputCallback(port, buffer);
}

void MMALH264Encoder::MMALInputCallback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer)
{
  mmal_buffer_header_release(buffer);
  //vcos_semaphore_post(&semaphore_);
}

void MMALH264Encoder::MMALOutputCallbackFunction(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer)
{
  ((MMALH264Encoder *)port->userdata)->MMALOutputCallback(port, buffer);
}

void MMALH264Encoder::MMALOutputCallback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer)
{
  mmal_queue_put(queue_, buffer);
  vcos_semaphore_post(&semaphore_);
}

int32_t MMALH264Encoder::RegisterEncodeCompleteCallback(
    webrtc::EncodedImageCallback *callback)
{
  callback_ = callback;
  return WEBRTC_VIDEO_CODEC_OK;
}

void MMALH264Encoder::SetRates(const RateControlParameters &parameters)
{
  if (encoder_ == nullptr)
    return;
  if (parameters.bitrate.get_sum_bps() <= 0 || parameters.framerate_fps <= 0)
    return;

  target_bitrate_bps_ = parameters.bitrate.get_sum_bps();
  bitrate_adjuster_.SetTargetBitrateBps(target_bitrate_bps_);
  SetBitrateBps(bitrate_adjuster_.GetAdjustedBitrateBps());
  return;
}

void MMALH264Encoder::SetBitrateBps(uint32_t bitrate_bps)
{
  if (configured_bitrate_bps_ == bitrate_bps)
  {
    return;
  }
  RTC_LOG(LS_INFO) << "SetBitrateBps " << bitrate_bps << "bit/sec";
  if (mmal_port_parameter_set_uint32(encoder_->output[0], MMAL_PARAMETER_VIDEO_BIT_RATE, bitrate_bps) != MMAL_SUCCESS)
  {
    RTC_LOG(LS_ERROR) << "Failed to set bitrate";
    return;
  }
  configured_bitrate_bps_ = bitrate_bps;
}

webrtc::VideoEncoder::EncoderInfo MMALH264Encoder::GetEncoderInfo() const
{
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
    const webrtc::VideoFrame &input_frame,
    const std::vector<webrtc::VideoFrameType> *frame_types)
{
  std::chrono::system_clock::time_point start, end;
  if (encoder_ == nullptr)
  {
    return WEBRTC_VIDEO_CODEC_UNINITIALIZED;
  }
  if (!callback_)
  {
    RTC_LOG(LS_WARNING) << "InitEncode() has been called, but a callback function "
                        << "has not been set with RegisterEncodeCompleteCallback()";
    return WEBRTC_VIDEO_CODEC_UNINITIALIZED;
  }

  rtc::scoped_refptr<const webrtc::I420BufferInterface> frame_buffer;
  frame_buffer = input_frame.video_frame_buffer()->ToI420();

  if (frame_buffer->width() != configured_width_ || frame_buffer->height() != configured_height_)
  {
    RTC_LOG(LS_INFO) << "Encoder reinitialized from " << configured_width_
                     << "x" << configured_height_ << " to "
                     << frame_buffer->width() << "x" << frame_buffer->height()
                     << " strideY:" << frame_buffer->StrideY()
                     << " strideU:" << frame_buffer->StrideU()
                     << " strideV:" << frame_buffer->StrideV();
    MMALRelease();
    if (MMALConfigure() != WEBRTC_VIDEO_CODEC_OK)
    {
      RTC_LOG(LS_ERROR) << "Failed to MMALConfigure";
      return WEBRTC_VIDEO_CODEC_ERROR;
    }
  }

  bool force_key_frame = false;
  if (frame_types != nullptr)
  {
    // We only support a single stream.
    RTC_DCHECK_EQ(frame_types->size(), static_cast<size_t>(1));
    // Skip frame?
    if ((*frame_types)[0] == webrtc::VideoFrameType::kEmptyFrame)
    {
      return WEBRTC_VIDEO_CODEC_OK;
    }
    // Force key frame?
    force_key_frame = (*frame_types)[0] == webrtc::VideoFrameType::kVideoFrameKey;
  }

  if (force_key_frame)
  {
    if (mmal_port_parameter_set_boolean(encoder_->output[0], MMAL_PARAMETER_VIDEO_REQUEST_I_FRAME, MMAL_TRUE) != MMAL_SUCCESS)
    {
      RTC_LOG(LS_ERROR) << "Failed to request I frame";
    }
  }

  SetBitrateBps(bitrate_adjuster_.GetAdjustedBitrateBps());

  MMAL_BUFFER_HEADER_T *buffer;
  while ((buffer = mmal_queue_get(pool_out_->queue)) != nullptr)
  {
    if (mmal_port_send_buffer(encoder_->output[0], buffer) != MMAL_SUCCESS)
    {
      RTC_LOG(LS_ERROR) << "Failed to send output buffer";
      return WEBRTC_VIDEO_CODEC_ERROR;
    }
    //RTC_LOG(LS_ERROR) << "get empty output buffer";
  }

  while ((buffer = mmal_queue_get(pool_in_->queue)) != nullptr)
  {
    size_t offset = 0;
    for (size_t i = 0; i < frame_buffer->height(); i++)
    {
      memcpy(buffer->data + offset,
             frame_buffer->DataY() + (frame_buffer->StrideY() * i),
             frame_buffer->width());
      offset += stride_width_;
    }
    offset = 0;
    size_t offset_y = stride_width_ * stride_height_;
    size_t width_uv = stride_width_ / 2;
    size_t offset_v = (stride_height_ / 2) * width_uv;
    for (size_t i = 0; i < ((frame_buffer->height() + 1) / 2); i++)
    {
      memcpy(buffer->data + offset_y + offset,
             frame_buffer->DataU() + (frame_buffer->StrideU() * i),
             width_uv);
      memcpy(buffer->data + offset_y + offset_v + offset,
             frame_buffer->DataV() + (frame_buffer->StrideV() * i),
             width_uv);
      offset += width_uv;
    }

    buffer->length = buffer->alloc_size = stride_width_ * stride_height_ * 3 / 2;
    //RTC_LOG(LS_ERROR) << "buffer->length: " << buffer->length;
    buffer->pts = MMAL_TIME_UNKNOWN; //input_frame.timestamp_us();
    buffer->offset = 0;
    if (mmal_port_send_buffer(encoder_->input[0], buffer) != MMAL_SUCCESS)
    {
      RTC_LOG(LS_ERROR) << "Failed to send input buffer";
      return WEBRTC_VIDEO_CODEC_ERROR;
    }
  }

  start = std::chrono::system_clock::now();
  vcos_semaphore_wait(&semaphore_);

  if ((buffer = mmal_queue_get(queue_)) == nullptr)
  {
    RTC_LOG(LS_ERROR) << "Failed to get output buffer";
    return WEBRTC_VIDEO_CODEC_ERROR;
  }

  end = std::chrono::system_clock::now();
  RTC_LOG(LS_INFO) << "duration: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

  encoded_image_._encodedWidth = frame_buffer->width();
  encoded_image_._encodedHeight = frame_buffer->height();
  encoded_image_.SetTimestamp(input_frame.timestamp());
  encoded_image_.ntp_time_ms_ = input_frame.ntp_time_ms();
  encoded_image_.capture_time_ms_ = input_frame.render_time_ms();
  encoded_image_.rotation_ = input_frame.rotation();
  encoded_image_.SetColorSpace(input_frame.color_space());

  int32_t result = SendFrame(buffer->data, buffer->length);
  mmal_buffer_header_release(buffer);
  return result;
}

int32_t MMALH264Encoder::SendFrame(unsigned char *buffer, size_t size)
{
  encoded_image_.set_buffer(buffer, size);
  encoded_image_.set_size(size);
  encoded_image_._frameType = webrtc::VideoFrameType::kVideoFrameDelta;
  //RTC_LOG(LS_ERROR) << __FUNCTION__ << " size:" << size;

  uint8_t zero_count = 0;
  size_t nal_start_idx = 0;
  std::vector<nal_entry> nals;
  for (size_t i = 0; i < size; i++)
  {
    uint8_t data = buffer[i];
    //if (i < 100) printf(" %02x", data);
    if ((i != 0) && (i == nal_start_idx))
    {
      //printf("-header");
      if ((data & 0x1F) == 0x05)
      {
        //printf("-IDR(%02x)", (data & 0x1F));
        encoded_image_._frameType = webrtc::VideoFrameType::kVideoFrameKey;
      }
    }
    if (data == 0x01 && zero_count == 3)
    {
      if (nal_start_idx != 0)
      {
        nals.push_back({nal_start_idx, i - nal_start_idx + 1 - 4});
        //printf(" nal_size: %d ", i - nal_start_idx + 1 - 4);
      }
      nal_start_idx = i + 1;
      //printf(" nal_start_idx: %d\n", nal_start_idx);
    }
    if (data == 0x00)
    {
      zero_count++;
    }
    else
    {
      zero_count = 0;
    }
  }
  if (nal_start_idx != 0)
  {
    nals.push_back({nal_start_idx, size - nal_start_idx});
    //printf(" nal_size: %d size: %d \n", size - nal_start_idx , size);
  }
  //printf("\n");

  //RTC_LOG(LS_ERROR) << __FUNCTION__ << "  nals.size():" << nals.size();

  webrtc::RTPFragmentationHeader frag_header;
  frag_header.VerifyAndAllocateFragmentationHeader(nals.size());
  for (size_t i = 0; i < nals.size(); i++)
  {
    frag_header.fragmentationOffset[i] = nals[i].offset;
    frag_header.fragmentationLength[i] = nals[i].size;
    frag_header.fragmentationPlType[i] = 0;
    frag_header.fragmentationTimeDiff[i] = 0;
    //RTC_LOG(LS_ERROR) << __FUNCTION__ << " i:" << i << " offset:" << nals[i].offset << " size:" << nals[i].size;
  }

  webrtc::CodecSpecificInfo codec_specific;
  codec_specific.codecType = webrtc::kVideoCodecH264;
  codec_specific.codecSpecific.H264.packetization_mode = webrtc::H264PacketizationMode::NonInterleaved;

  h264_bitstream_parser_.ParseBitstream(buffer, size);
  h264_bitstream_parser_.GetLastSliceQp(&encoded_image_.qp_);

  webrtc::EncodedImageCallback::Result result = callback_->OnEncodedImage(encoded_image_, &codec_specific, &frag_header);
  if (result.error != webrtc::EncodedImageCallback::Result::OK)
  {
    RTC_LOG(LS_ERROR) << __FUNCTION__ << " OnEncodedImage failed error:" << result.error;
    return WEBRTC_VIDEO_CODEC_ERROR;
  }
  bitrate_adjuster_.Update(size);
  return WEBRTC_VIDEO_CODEC_OK;
}