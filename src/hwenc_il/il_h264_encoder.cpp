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

#include "il_h264_encoder.h"

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

ILH264Encoder::ILH264Encoder(const cricket::VideoCodec &codec)
    : callback_(nullptr),
      ilclient_(nullptr),
      video_encode_(nullptr),
      bitrate_adjuster_(.5, .95)
{
}

ILH264Encoder::~ILH264Encoder()
{
  Release();
}

int32_t ILH264Encoder::InitEncode(const webrtc::VideoCodec *codec_settings,
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
  RTC_DCHECK(!ilclient_);

  bcm_host_init();

  memset(list_, 0, sizeof(list_));

  // init ilclient.
  if ((ilclient_ = ilclient_init()) == nullptr)
  {
    // Failed to init ilclient.
    RTC_LOG(LS_ERROR) << "Failed to init ILCLIENT";
    RTC_DCHECK(!ilclient_);
    return WEBRTC_VIDEO_CODEC_ERROR;
  }

  RTC_DCHECK(ilclient_);
  if (OMX_Init() != OMX_ErrorNone)
  {
    // Failed to init ilclient.
    RTC_LOG(LS_ERROR) << "Failed to init OMX";
    ilclient_destroy(ilclient_);
    return WEBRTC_VIDEO_CODEC_ERROR;
  }

  if (ilclient_create_component(ilclient_, &video_encode_, (char *)"video_encode",
                                (ILCLIENT_CREATE_FLAGS_T)(ILCLIENT_DISABLE_ALL_PORTS |
                                                          ILCLIENT_ENABLE_INPUT_BUFFERS |
                                                          ILCLIENT_ENABLE_OUTPUT_BUFFERS)) == -1)
  {
    RTC_LOG(LS_ERROR) << "Failed to create ilclient component";
    Release();
    return WEBRTC_VIDEO_CODEC_ERROR;
  }

  width_ = codec_settings->width;
  height_ = codec_settings->height;
  target_bitrate_bps_ = codec_settings->startBitrate * 1000;
  bitrate_adjuster_.SetTargetBitrateBps(target_bitrate_bps_);

  RTC_LOG(LS_ERROR) << "InitEncode " << (uint32_t)(codec_settings->maxFramerate) << "fps "
                    << (uint32_t)(target_bitrate_bps_) << "bit/sec";

  list_[0] = video_encode_;

  // Initialize encoded image. Default buffer size: size of unencoded data.
  const size_t new_capacity = CalcBufferSize(
      webrtc::VideoType::kI420, codec_settings->width, codec_settings->height);
  encoded_image_buffer_.reset(new uint8_t[new_capacity]);
  encoded_image_.set_buffer(encoded_image_buffer_.get(), new_capacity);
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

int32_t ILH264Encoder::Release()
{
  if (ilclient_)
  {
    if (video_encode_)
    {
      ilclient_disable_port_buffers(video_encode_, 200, NULL, NULL, NULL);
      ilclient_disable_port_buffers(video_encode_, 201, NULL, NULL, NULL);
      video_encode_ = nullptr;
    }
    ilclient_state_transition(list_, OMX_StateIdle);
    ilclient_state_transition(list_, OMX_StateLoaded);
    ilclient_cleanup_components(list_);
    list_[0] = nullptr;
    if (OMX_Deinit() != OMX_ErrorNone)
    {
      RTC_LOG(LS_WARNING) << "Failed to deinit OMX";
    }
    ilclient_destroy(ilclient_);
    ilclient_ = nullptr;
  }
  encoded_image_buffer_.reset();
  return WEBRTC_VIDEO_CODEC_OK;
}

int32_t ILH264Encoder::OMX_Configure()
{
  OMX_ERRORTYPE err;

  OMX_PARAM_PORTDEFINITIONTYPE defin;
  memset(&defin, 0, sizeof(OMX_PARAM_PORTDEFINITIONTYPE));
  defin.nSize = sizeof(OMX_PARAM_PORTDEFINITIONTYPE);
  defin.nVersion.nVersion = OMX_VERSION;
  defin.nPortIndex = 200;
  // Initialization parameters.
  err = OMX_GetParameter(ILC_GET_HANDLE(video_encode_), OMX_IndexParamPortDefinition, &defin);
  if (err != OMX_ErrorNone)
  {
    RTC_LOG(LS_ERROR) << "Failed to OMX_GetParameter for video_encode port 200  ErrorType: " << err;
    Release();
    return WEBRTC_VIDEO_CODEC_ERROR;
  }

  defin.format.video.nFrameWidth = width_;
  defin.format.video.nFrameHeight = height_;
  defin.format.video.xFramerate = kFramerate << 16;
  defin.format.video.nSliceHeight = defin.format.video.nFrameHeight;
  if (defin.nBufferAlignment)
  {
    defin.format.video.nStride =
        (defin.format.video.nFrameWidth + defin.nBufferAlignment - 1) &
        (~(defin.nBufferAlignment - 1));
  }
  else
  {
    defin.format.video.nStride = ROUND_UP_4(defin.format.video.nFrameWidth);
  }
  defin.format.video.eColorFormat = OMX_COLOR_FormatYUV420PackedPlanar;
  defin.nBufferSize = CalcBufferSize(webrtc::VideoType::kI420, width_, height_);

  err = OMX_SetParameter(ILC_GET_HANDLE(video_encode_), OMX_IndexParamPortDefinition, &defin);
  if (err != OMX_ErrorNone)
  {
    RTC_LOG(LS_ERROR) << "Failed to OMX_SetParameter for video_encode port 200  ErrorType: " << err;
    Release();
    return WEBRTC_VIDEO_CODEC_ERROR;
  }
  configured_width_ = width_;
  configured_height_ = height_;

  OMX_PARAM_PORTDEFINITIONTYPE defout;
  memset(&defout, 0, sizeof(OMX_PARAM_PORTDEFINITIONTYPE));
  defout.nSize = sizeof(OMX_PARAM_PORTDEFINITIONTYPE);
  defout.nVersion.nVersion = OMX_VERSION;
  defout.nPortIndex = 201;
  // Initialization parameters.
  err = OMX_GetParameter(ILC_GET_HANDLE(video_encode_), OMX_IndexParamPortDefinition, &defout);
  if (err != OMX_ErrorNone)
  {
    RTC_LOG(LS_ERROR) << "Failed to OMX_GetParameter for video_encode port 201  ErrorType: " << err;
    Release();
    return WEBRTC_VIDEO_CODEC_ERROR;
  }

  defout.nBufferSize = defin.nBufferSize;

  err = OMX_SetParameter(ILC_GET_HANDLE(video_encode_), OMX_IndexParamPortDefinition, &defout);
  if (err != OMX_ErrorNone)
  {
    RTC_LOG(LS_ERROR) << "Failed to OMX_SetParameter for video_encode port 201  ErrorType: " << err;
    Release();
    return WEBRTC_VIDEO_CODEC_ERROR;
  }

  OMX_VIDEO_PARAM_PORTFORMATTYPE format;
  memset(&format, 0, sizeof(OMX_VIDEO_PARAM_PORTFORMATTYPE));
  format.nSize = sizeof(OMX_VIDEO_PARAM_PORTFORMATTYPE);
  format.nVersion.nVersion = OMX_VERSION;
  format.nPortIndex = 201;
  format.eCompressionFormat = OMX_VIDEO_CodingAVC;
  err = OMX_SetParameter(ILC_GET_HANDLE(video_encode_), OMX_IndexParamVideoPortFormat, &format);
  if (err != OMX_ErrorNone)
  {
    RTC_LOG(LS_ERROR) << "Failed to OMX_SetParameter for video_encode port 201  ErrorType: " << err;
    Release();
    return WEBRTC_VIDEO_CODEC_ERROR;
  }

  OMX_VIDEO_PARAM_PROFILELEVELTYPE profile;
  memset(&profile, 0, sizeof(OMX_VIDEO_PARAM_PROFILELEVELTYPE));
  profile.nSize = sizeof(OMX_VIDEO_PARAM_PROFILELEVELTYPE);
  profile.nVersion.nVersion = OMX_VERSION;
  profile.nPortIndex = 201;
  profile.eProfile = OMX_VIDEO_AVCProfileBaseline;
  profile.eLevel = OMX_VIDEO_AVCLevel31;
  profile.nProfileIndex = 1;
  err = OMX_SetParameter(ILC_GET_HANDLE(video_encode_), OMX_IndexParamVideoProfileLevelCurrent, &profile);
  if (err != OMX_ErrorNone)
  {
    RTC_LOG(LS_ERROR) << "Failed to OMX_SetParameter for Video Profile Level  ErrorType: " << err;
    Release();
    return WEBRTC_VIDEO_CODEC_ERROR;
  }

  OMX_VIDEO_PARAM_AVCTYPE avcType;
  memset(&avcType, 0, sizeof(OMX_VIDEO_PARAM_AVCTYPE));
  avcType.nSize = sizeof(OMX_VIDEO_PARAM_PORTFORMATTYPE);
  avcType.nVersion.nVersion = OMX_VERSION;
  avcType.nPortIndex = 201;
  err = OMX_GetParameter(ILC_GET_HANDLE(video_encode_), OMX_IndexParamVideoAvc, &avcType);
  if (err != OMX_ErrorNone)
  {
    RTC_LOG(LS_ERROR) << "Failed to OMX_GetParameter for AVC Type  ErrorType: " << err;
    Release();
    return WEBRTC_VIDEO_CODEC_ERROR;
  }

  avcType.nAllowedPictureTypes = OMX_VIDEO_PictureTypeI | OMX_VIDEO_PictureTypeP;
  avcType.eProfile = OMX_VIDEO_AVCProfileBaseline;
  avcType.eLevel = OMX_VIDEO_AVCLevel31;
  avcType.bEntropyCodingCABAC = OMX_FALSE;

  err = OMX_SetParameter(ILC_GET_HANDLE(video_encode_), OMX_IndexParamVideoAvc, &avcType);
  if (err != OMX_ErrorNone)
  {
    RTC_LOG(LS_ERROR) << "Failed to OMX_SetParameter for AVC Type  ErrorType: " << err;
    Release();
    return WEBRTC_VIDEO_CODEC_ERROR;
  }

  OMX_CONFIG_PORTBOOLEANTYPE inlineHeader;
  memset(&inlineHeader, 0, sizeof(OMX_CONFIG_PORTBOOLEANTYPE));
  inlineHeader.nSize = sizeof(OMX_PARAM_PORTDEFINITIONTYPE);
  inlineHeader.nVersion.nVersion = OMX_VERSION;
  inlineHeader.nPortIndex = 201;
  inlineHeader.bEnabled = OMX_TRUE;
  err = OMX_SetParameter(ILC_GET_HANDLE(video_encode_),
                         OMX_IndexParamBrcmVideoAVCInlineHeaderEnable, &inlineHeader);
  if (err != OMX_ErrorNone)
  {
    RTC_LOG(LS_ERROR) << "Failed to OMX_SetParameter for InlineHeader Enable  ErrorType: " << err;
    Release();
    return WEBRTC_VIDEO_CODEC_ERROR;
  }

  OMX_VIDEO_PARAM_BITRATETYPE bitrateType;
  // set current bitrate to 1Mbit
  memset(&bitrateType, 0, sizeof(OMX_VIDEO_PARAM_BITRATETYPE));
  bitrateType.nSize = sizeof(OMX_VIDEO_PARAM_BITRATETYPE);
  bitrateType.nVersion.nVersion = OMX_VERSION;
  bitrateType.eControlRate = OMX_Video_ControlRateConstantSkipFrames;
  bitrateType.nTargetBitrate = target_bitrate_bps_;
  bitrateType.nPortIndex = 201;
  err = OMX_SetParameter(ILC_GET_HANDLE(video_encode_), OMX_IndexParamVideoBitrate, &bitrateType);
  if (err != OMX_ErrorNone)
  {
    RTC_LOG(LS_ERROR) << "Failed to OMX_SetParameter for bitrate for video_encode port 201  ErrorType: " << err;
    Release();
    return WEBRTC_VIDEO_CODEC_ERROR;
  }

  if (ilclient_change_component_state(video_encode_, OMX_StateIdle) == -1)
  {
    RTC_LOG(LS_ERROR) << "Failed to change component state for OMX_StateIdle";
    Release();
    return WEBRTC_VIDEO_CODEC_ERROR;
  }

  if (ilclient_enable_port_buffers(video_encode_, 200, NULL, NULL, NULL) != 0)
  {
    RTC_LOG(LS_ERROR) << "Failed to enable port buffers for 200";
    Release();
    return WEBRTC_VIDEO_CODEC_ERROR;
  }

  if (ilclient_enable_port_buffers(video_encode_, 201, NULL, NULL, NULL) != 0)
  {
    RTC_LOG(LS_ERROR) << "Failed to enable port buffers for 201";
    Release();
    return WEBRTC_VIDEO_CODEC_ERROR;
  }

  ilclient_change_component_state(video_encode_, OMX_StateExecuting);

  return WEBRTC_VIDEO_CODEC_OK;
}

void ILH264Encoder::OMX_Release()
{
  if (ilclient_)
  {
    OMX_STATETYPE state;
    OMX_ERRORTYPE err = OMX_GetState(ILC_GET_HANDLE(video_encode_), &state);
    vc_assert(err == OMX_ErrorNone);
    if (err != OMX_ErrorNone || state == OMX_StateIdle || state == OMX_StateExecuting)
    {
      if (video_encode_)
      {
        if (state == OMX_StateIdle || state == OMX_StateExecuting)
        {
          ilclient_disable_port_buffers(video_encode_, 200, NULL, NULL, NULL);
          ilclient_disable_port_buffers(video_encode_, 201, NULL, NULL, NULL);
        }
      }
      ilclient_state_transition(list_, OMX_StateIdle);
      ilclient_state_transition(list_, OMX_StateLoaded);
    }
  }
}

int32_t ILH264Encoder::RegisterEncodeCompleteCallback(
    webrtc::EncodedImageCallback *callback)
{
  callback_ = callback;
  return WEBRTC_VIDEO_CODEC_OK;
}

void ILH264Encoder::SetRates(const RateControlParameters &parameters)
{
  if (parameters.bitrate.get_sum_bps() <= 0 || parameters.framerate_fps <= 0)
    return;

  target_bitrate_bps_ = parameters.bitrate.get_sum_bps();
  bitrate_adjuster_.SetTargetBitrateBps(target_bitrate_bps_);
  SetBitrateBps(bitrate_adjuster_.GetAdjustedBitrateBps());
  return;
}

void ILH264Encoder::SetBitrateBps(uint32_t bitrate_bps)
{
  if (configured_bitrate_bps_ == bitrate_bps)
  {
    return;
  }
  RTC_LOG(LS_INFO) << "SetBitrateBps" << bitrate_bps << "bit/sec";
  OMX_VIDEO_CONFIG_BITRATETYPE bitrateType;
  memset(&bitrateType, 0, sizeof(OMX_VIDEO_CONFIG_BITRATETYPE));
  bitrateType.nSize = sizeof(OMX_VIDEO_CONFIG_BITRATETYPE);
  bitrateType.nVersion.nVersion = OMX_VERSION;
  bitrateType.nPortIndex = 201;
  bitrateType.nEncodeBitrate = bitrate_bps;
  OMX_ERRORTYPE err = OMX_SetConfig(ILC_GET_HANDLE(video_encode_),
                                    OMX_IndexConfigVideoBitrate, &bitrateType);
  if (err != OMX_ErrorNone)
  {
    RTC_LOG(LS_ERROR) << "Failed to set bitrate parameter ErrorType: " << err;
  }
  configured_bitrate_bps_ = bitrate_bps;
}

webrtc::VideoEncoder::EncoderInfo ILH264Encoder::GetEncoderInfo() const
{
  EncoderInfo info;
  info.supports_native_handle = true;
  info.implementation_name = "Open MAX IL H264";
  info.scaling_settings =
      VideoEncoder::ScalingSettings(kLowH264QpThreshold, kHighH264QpThreshold);
  info.is_hardware_accelerated = true;
  info.has_internal_source = false;
  return info;
}

int32_t ILH264Encoder::Encode(
    const webrtc::VideoFrame &input_frame,
    const std::vector<webrtc::VideoFrameType> *frame_types)
{
  if (ilclient_ == nullptr)
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
                     << frame_buffer->width() << "x" << frame_buffer->height();
    OMX_Release();
    OMX_Configure();
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

  OMX_ERRORTYPE err;
  if (force_key_frame)
  {
    OMX_CONFIG_BOOLEANTYPE requestIFrame;
    memset(&requestIFrame, 0, sizeof(OMX_CONFIG_BOOLEANTYPE));
    requestIFrame.nSize = sizeof(OMX_VIDEO_PARAM_BITRATETYPE);
    requestIFrame.nVersion.nVersion = OMX_VERSION;
    requestIFrame.bEnabled = OMX_TRUE;
    err = OMX_SetConfig(ILC_GET_HANDLE(video_encode_),
                        OMX_IndexConfigBrcmVideoRequestIFrame, &requestIFrame);
    if (err != OMX_ErrorNone)
    {
      RTC_LOG(LS_ERROR) << "#####Failed to OMX_SetConfig for Request IFrame ErrorType: " << err;
    }
  }

  SetBitrateBps(bitrate_adjuster_.GetAdjustedBitrateBps());

  OMX_BUFFERHEADERTYPE *buf;
  buf = ilclient_get_input_buffer(video_encode_, 200, 1);
  if (buf == NULL)
  {
    RTC_LOG(LS_ERROR) << "Failed to get input buffer";
    return WEBRTC_VIDEO_CODEC_ERROR;
  }

  buf->nFilledLen = I420DataSize(*frame_buffer);
  memcpy(buf->pBuffer, frame_buffer->DataY(), buf->nFilledLen);
  buf->nTimeStamp.nLowPart = input_frame.timestamp() * 1000ll;
  buf->nTimeStamp.nHighPart = (input_frame.timestamp() * 1000ll) >> 32;

  err = OMX_EmptyThisBuffer(ILC_GET_HANDLE(video_encode_), buf);
  if (err != OMX_ErrorNone)
  {
    RTC_LOG(LS_ERROR) << "Failed to empty input buffer";
    return WEBRTC_VIDEO_CODEC_ERROR;
  }

  encoded_image_._encodedWidth = frame_buffer->width();
  encoded_image_._encodedHeight = frame_buffer->height();
  encoded_image_.SetTimestamp(input_frame.timestamp());
  encoded_image_.ntp_time_ms_ = input_frame.ntp_time_ms();
  encoded_image_.capture_time_ms_ = input_frame.render_time_ms();
  encoded_image_.rotation_ = input_frame.rotation();
  encoded_image_.SetColorSpace(input_frame.color_space());

  OMX_BUFFERHEADERTYPE *out;
  out = ilclient_get_output_buffer(video_encode_, 201, 1);
  int32_t result = WEBRTC_VIDEO_CODEC_ERROR;
  if (out == nullptr)
  {
    RTC_LOG(LS_ERROR) << "Failed to get output buffer";
  }
  else
  {
    result = SendFrame(out);
  }
  out->nFilledLen = 0;
  err = OMX_FillThisBuffer(ILC_GET_HANDLE(video_encode_), out);
  if (err != OMX_ErrorNone)
  {
    RTC_LOG(LS_ERROR) << "Failed to fill output buffer";
  }
  return result;
}

int32_t ILH264Encoder::SendFrame(OMX_BUFFERHEADERTYPE *out)
{
  unsigned char *buffer = out->pBuffer + out->nOffset;
  size_t size = out->nFilledLen;

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