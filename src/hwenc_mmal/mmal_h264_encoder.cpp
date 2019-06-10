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

const int kLowH264QpThreshold = 24;
const int kHighH264QpThreshold = 37;

int I420DataSize(const webrtc::I420BufferInterface &frame_buffer)
{
  return frame_buffer.StrideY() * frame_buffer.height() + (frame_buffer.StrideU() + frame_buffer.StrideV()) * ((frame_buffer.height() + 1) / 2);
}

struct RTCFrameEncodeParams {
  RTCFrameEncodeParams(int32_t w,
                       int32_t h,
                       int64_t rtms,
                       int64_t ntpms,
                       int64_t ts,
                       webrtc::VideoRotation r,
                       absl::optional<webrtc::ColorSpace> c)
      : width(w), height(h), render_time_ms(rtms), ntp_time_ms(ntpms), timestamp(ts), rotation(r), color_space(c) {}

  int32_t width;
  int32_t height;
  int64_t render_time_ms;
  int64_t ntp_time_ms;
  int64_t timestamp;
  webrtc::VideoRotation rotation;
  absl::optional<webrtc::ColorSpace> color_space;
};

} // namespace

MMALH264Encoder::MMALH264Encoder(const cricket::VideoCodec &codec)
    : callback_(nullptr),
      encoder_(nullptr),
      queue_(nullptr),
      pool_out_(nullptr),
      bitrate_adjuster_(.5, .95),
      configured_framerate_(30),
      configured_width_(0),
      configured_height_(0),
      encoded_buffer_length_(0),
      initial_timestamp_(0)
{
  start_ = std::chrono::system_clock::now();
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
  framerate_ = codec_settings->maxFramerate;
  if (framerate_ > 30)
  {
    framerate_ = 30;
  }

  RTC_LOG(LS_ERROR) << "InitEncode " << framerate_ << "fps "
                    << target_bitrate_bps_ << "bit/sec";

  // Initialize encoded image. Default buffer size: size of unencoded data.
  encoded_image_._completeFrame = true;
  encoded_image_._encodedWidth = 0;
  encoded_image_._encodedHeight = 0;
  encoded_image_.set_size(0);
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
  format_in->es->video.frame_rate.num = framerate_;
  format_in->es->video.frame_rate.den = 1;

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
  encoder_->output[0]->format->bitrate = bitrate_adjuster_.GetAdjustedBitrateBps();

  if (mmal_port_format_commit(encoder_->output[0]) != MMAL_SUCCESS)
  {
    RTC_LOG(LS_ERROR) << "Failed to commit output port format";
    return WEBRTC_VIDEO_CODEC_ERROR;
  }

  /*if (mmal_port_parameter_set_boolean(encoder_->input[0], MMAL_PARAMETER_ZERO_COPY, MMAL_TRUE) != MMAL_SUCCESS)
  {
    RTC_LOG(LS_ERROR) << "Failed to set input zero copy";
    return WEBRTC_VIDEO_CODEC_ERROR;
  }

  if (mmal_port_parameter_set_boolean(encoder_->output[0], MMAL_PARAMETER_ZERO_COPY, MMAL_TRUE) != MMAL_SUCCESS)
  {
    RTC_LOG(LS_ERROR) << "Failed to set output zero copy";
    return WEBRTC_VIDEO_CODEC_ERROR;
  }*/

  MMAL_PARAMETER_VIDEO_PROFILE_T video_profile;
  video_profile.hdr.id = MMAL_PARAMETER_PROFILE;
  video_profile.hdr.size = sizeof(video_profile);

  video_profile.profile[0].profile = MMAL_VIDEO_PROFILE_H264_HIGH;
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

  /*if (mmal_port_parameter_set_boolean(encoder_->output[0], MMAL_PARAMETER_VIDEO_ENCODE_H264_LOW_LATENCY, MMAL_TRUE) != MMAL_SUCCESS)
  {
    RTC_LOG(LS_ERROR) << "Failed to set enable low latency";
    return WEBRTC_VIDEO_CODEC_ERROR;
  }*/

  /*if (mmal_port_parameter_set_uint32(encoder_->output[0], MMAL_PARAMETER_RATECONTROL, MMAL_VIDEO_RATECONTROL_CONSTANT) != MMAL_SUCCESS)
  {
    RTC_LOG(LS_ERROR) << "Failed to set rate control";
    return WEBRTC_VIDEO_CODEC_ERROR;
  }*/

  queue_ = mmal_queue_create();

  encoder_->input[0]->buffer_size = encoder_->input[0]->buffer_size_recommended * 2;
  if (encoder_->input[0]->buffer_size < encoder_->input[0]->buffer_size_min)
    encoder_->input[0]->buffer_size = encoder_->input[0]->buffer_size_min;
  encoder_->input[0]->buffer_num = 1;
  encoder_->input[0]->userdata = (MMAL_PORT_USERDATA_T *)this;

  RTC_LOG(LS_ERROR) << "input buffer_size:" << encoder_->input[0]->buffer_size;

  if (mmal_port_enable(encoder_->input[0], MMALInputCallbackFunction) != MMAL_SUCCESS)
  {
    RTC_LOG(LS_ERROR) << "Failed to enable input port";
    return WEBRTC_VIDEO_CODEC_ERROR;
  }
  pool_in_ = mmal_port_pool_create(encoder_->input[0], encoder_->input[0]->buffer_num, encoder_->input[0]->buffer_size);

  encoder_->output[0]->buffer_size = encoder_->output[0]->buffer_size_recommended * 5;
  if (encoder_->output[0]->buffer_size < encoder_->output[0]->buffer_size_min)
    encoder_->output[0]->buffer_size = encoder_->output[0]->buffer_size_min;
  encoder_->output[0]->buffer_num = 8;
  encoder_->output[0]->userdata = (MMAL_PORT_USERDATA_T *)this;

  encoded_image_buffer_.reset(new uint8_t[encoder_->output[0]->buffer_size]);
  RTC_LOG(LS_ERROR) << "output buffer_size:" << encoder_->output[0]->buffer_size;

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

  configured_framerate_ = framerate_;
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
    if (pool_in_ != nullptr)
    {
      mmal_pool_destroy(pool_in_);
      pool_in_ = nullptr;
    }
    if (pool_out_ != nullptr)
    {
      if (!vcos_verify(mmal_queue_length(pool_out_->queue) == pool_out_->headers_num))
      {
        RTC_LOG(LS_ERROR) << "Failed to release all output buffers";
      }
      mmal_pool_destroy(pool_out_);
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
}

void MMALH264Encoder::MMALOutputCallbackFunction(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer)
{
  ((MMALH264Encoder *)port->userdata)->MMALOutputCallback(port, buffer);
}

void MMALH264Encoder::MMALOutputCallback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer)
{
  std::chrono::system_clock::time_point end = std::chrono::system_clock::now();
  RTC_LOG(LS_INFO) << "Encode duration: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start_).count();
  RTCFrameEncodeParams* params =(RTCFrameEncodeParams *)buffer->user_data;


  RTC_LOG(LS_ERROR) << "timestamp:" << params->timestamp
                    << " duration:" << buffer->pts - initial_timestamp_
                    << " pts:" << buffer->pts
                    << " flags:" << buffer->flags
                    << " planes:" << buffer->type->video.planes
                    << " length:" << buffer->length;
  if (buffer->length == 0) {
    delete params;
    mmal_buffer_header_release(buffer);
    return;
  }

  if (buffer->flags & MMAL_BUFFER_HEADER_FLAG_CONFIG) {
    memcpy(encoded_image_buffer_.get(), buffer->data, buffer->length);
    encoded_buffer_length_ = buffer->length;
    delete params;
    mmal_buffer_header_release(buffer);
    RTC_LOG(LS_ERROR) << "MMAL_BUFFER_HEADER_FLAG_CONFIG";
    return;
  }
  initial_timestamp_ = buffer->pts;
  
  encoded_image_._encodedWidth = params->width;
  encoded_image_._encodedHeight = params->height;
  encoded_image_.capture_time_ms_ = params->render_time_ms;
  encoded_image_.ntp_time_ms_ = params->ntp_time_ms;
  encoded_image_.SetTimestamp(buffer->pts);
  encoded_image_.rotation_ = params->rotation;
  encoded_image_.SetColorSpace(params->color_space);

  delete params;

  if (encoded_buffer_length_ == 0)
  {
    SendFrame(buffer->data, buffer->length);
  }
  else
  {
    memcpy(encoded_image_buffer_.get() + encoded_buffer_length_, buffer->data, buffer->length);
    encoded_buffer_length_ += buffer->length;
    SendFrame(encoded_image_buffer_.get(), encoded_buffer_length_);
    encoded_buffer_length_ = 0;
  }
  
  mmal_buffer_header_release(buffer);
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

  RTC_LOG(LS_ERROR) << __FUNCTION__ 
                    << " framerate:" << parameters.framerate_fps
                    << " bitrate:" << parameters.bitrate.get_sum_bps();
  //framerate_ = parameters.framerate_fps;
  if (framerate_ > 30)
  {
    framerate_ = 30;
  }
  target_bitrate_bps_ = parameters.bitrate.get_sum_bps();
  bitrate_adjuster_.SetTargetBitrateBps(target_bitrate_bps_);
  return;
}

void MMALH264Encoder::SetBitrateBps(uint32_t bitrate_bps)
{
  if (bitrate_bps < 300000 || configured_bitrate_bps_ == bitrate_bps)
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

  if (frame_buffer->width() != configured_width_ ||
      frame_buffer->height() != configured_height_ ||
      framerate_ != configured_framerate_)
  {
    RTC_LOG(LS_INFO) << "Encoder reinitialized from " << configured_width_
                     << "x" << configured_height_ << " to "
                     << frame_buffer->width() << "x" << frame_buffer->height()
                     << " strideY:" << frame_buffer->StrideY()
                     << " strideU:" << frame_buffer->StrideU()
                     << " strideV:" << frame_buffer->StrideV()
                     << " framerate:" << framerate_;
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
    buffer->user_data = new RTCFrameEncodeParams(frame_buffer->width(),
                                                 frame_buffer->height(),
                                                 input_frame.render_time_ms(),
                                                 input_frame.ntp_time_ms(),
                                                 input_frame.timestamp(),
                                                 input_frame.rotation(),
                                                 input_frame.color_space());
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
             frame_buffer->DataY() + (frame_buffer->width() * i),
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
    RTC_LOG(LS_ERROR) << "buffer->length: " << buffer->length;
    buffer->pts = buffer->dts = input_frame.timestamp();
    buffer->offset = 0;
    buffer->flags = MMAL_BUFFER_HEADER_FLAG_FRAME;
    if (mmal_port_send_buffer(encoder_->input[0], buffer) != MMAL_SUCCESS)
    {
      RTC_LOG(LS_ERROR) << "Failed to send input buffer";
      return WEBRTC_VIDEO_CODEC_ERROR;
    }
  }

  //vcos_semaphore_wait(&semaphore_);
  start_ = std::chrono::system_clock::now();
  return WEBRTC_VIDEO_CODEC_OK;
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
    RTC_LOG(LS_ERROR) << __FUNCTION__ 
                      << " OnEncodedImage failed error:" << result.error
                      << " size:" << size
                      << " _frameType:" << encoded_image_._frameType;
    return WEBRTC_VIDEO_CODEC_ERROR;
  }
  bitrate_adjuster_.Update(size);
  return WEBRTC_VIDEO_CODEC_OK;
}