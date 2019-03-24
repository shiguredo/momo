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

const uint32_t kFramerate = 30;
const int kLowH264QpThreshold = 28;
const int kHighH264QpThreshold = 39;

// Used by histograms. Values of entries should not be changed.
enum ILH264EncoderEvent
{
  kH264EncoderEventInit = 0,
  kH264EncoderEventError = 1,
  kH264EncoderEventMax = 16,
};

} // namespace

int I420DataSize(const webrtc::I420BufferInterface &frame_buffer)
{
  return frame_buffer.StrideY() * frame_buffer.height() + (frame_buffer.StrideU() + frame_buffer.StrideV()) * ((frame_buffer.height() + 1) / 2);
}

// https://android.googlesource.com/platform/frameworks/av/+/master/media/libstagefright/avc_utils.cpp#257
static bool getNextNALUnit(const uint8_t **_data, size_t *_size,
                           const uint8_t **nalStart, size_t *nalSize,
                           bool startCodeFollows)
{
  const uint8_t *data = *_data;
  size_t size = *_size;

  *nalStart = NULL;
  *nalSize = 0;

  if (size < 3)
  {
    return false;
  }

  size_t offset = 0;

  // A valid startcode consists of at least two 0x00 bytes followed by 0x01.
  for (; offset + 2 < size; ++offset)
  {
    if (data[offset + 2] == 0x01 && data[offset] == 0x00 && data[offset + 1] == 0x00)
    {
      break;
    }
  }
  if (offset + 2 >= size)
  {
    *_data = &data[offset];
    *_size = 2;
    return false;
  }
  offset += 3;

  size_t startOffset = offset;

  for (;;)
  {
    while (offset < size && data[offset] != 0x01)
    {
      ++offset;
    }

    if (offset == size)
    {
      if (startCodeFollows)
      {
        offset = size + 2;
        break;
      }

      return false;
    }

    if (data[offset - 1] == 0x00 && data[offset - 2] == 0x00)
    {
      break;
    }

    ++offset;
  }

  size_t endOffset = offset - 2;
  while (endOffset > startOffset + 1 && data[endOffset - 1] == 0x00)
  {
    --endOffset;
  }

  *nalStart = &data[startOffset];
  *nalSize = endOffset - startOffset;

  if (offset + 2 < size)
  {
    *_data = &data[offset - 2];
    *_size = size - offset + 2;
  }
  else
  {
    *_data = NULL;
    *_size = 0;
  }

  return true;
}

ILH264Encoder::ILH264Encoder(const cricket::VideoCodec &codec)
    : callback_(nullptr),
      ilclient_(nullptr),
      video_encode_(nullptr),
      fill_buffer_event_(false, false),
      bitrate_adjuster_(.5, .95),
      packetization_mode_(webrtc::H264PacketizationMode::NonInterleaved),
      sps_length_(0),
      pps_length_(0),
      omx_configured_(false),
      omx_reconfigure_(false),
      drop_next_frame_(false) {}

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

  ilclient_set_fill_buffer_done_callback(ilclient_, FillBufferDoneFunction, this);

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

void ILH264Encoder::FillBufferDoneFunction(void *data, COMPONENT_T *comp)
{
  if (comp == static_cast<ILH264Encoder *>(data)->video_encode_)
  {
    static_cast<ILH264Encoder *>(data)->FillBufferDone();
  }
}

void ILH264Encoder::FillBufferDone()
{
  fill_buffer_event_.Set();
}

int32_t ILH264Encoder::RegisterEncodeCompleteCallback(
    webrtc::EncodedImageCallback *callback)
{
  callback_ = callback;
  return WEBRTC_VIDEO_CODEC_OK;
}

int32_t ILH264Encoder::SetRateAllocation(const webrtc::VideoBitrateAllocation &bitrate_allocation,
                                          uint32_t framerate)
{
  if (bitrate_allocation.get_sum_bps() <= 0 || framerate <= 0)
    return WEBRTC_VIDEO_CODEC_ERR_PARAMETER;

  target_bitrate_bps_ = bitrate_allocation.get_sum_bps();
  bitrate_adjuster_.SetTargetBitrateBps(target_bitrate_bps_);
  SetBitrateBps(bitrate_adjuster_.GetAdjustedBitrateBps());
  return WEBRTC_VIDEO_CODEC_OK;
}

void ILH264Encoder::SetBitrateBps(uint32_t bitrate_bps)
{
  if (encoder_bitrate_bps_ != bitrate_bps)
  {
    SetEncoderBitrateBps(bitrate_bps);
  }
}

void ILH264Encoder::SetEncoderBitrateBps(uint32_t bitrate_bps)
{
  RTC_LOG(LS_ERROR) << "SetEncoderBitrateBps　" << bitrate_bps << "bit/sec";
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
  encoder_bitrate_bps_ = bitrate_bps;
}

int32_t ILH264Encoder::Encode(
    const webrtc::VideoFrame &input_frame, const webrtc::CodecSpecificInfo *codec_specific_info,
    const std::vector<webrtc::FrameType> *frame_types)
{
  if (!IsInitialized())
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
    omx_reconfigure_ = true;
  }

  bool force_key_frame = false;

  if (!omx_configured_ || omx_reconfigure_)
  {
    if (omx_configured_)
    {
      omx_configured_ = false;
    }
    omx_reconfigure_ = false;
    OMX_Release();
    OMX_Configure();
    omx_configured_ = true;
  }

  if (frame_types != nullptr)
  {
    // We only support a single stream.
    RTC_DCHECK_EQ(frame_types->size(), static_cast<size_t>(1));
    // Skip frame?
    if ((*frame_types)[0] == webrtc::kEmptyFrame)
    {
      return WEBRTC_VIDEO_CODEC_OK;
    }
    // Force key frame?
    force_key_frame = (*frame_types)[0] == webrtc::kVideoFrameKey;
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
    drop_next_frame_ = true;
  }

  encoded_image_._encodedWidth = frame_buffer->width();
  encoded_image_._encodedHeight = frame_buffer->height();
  encoded_image_.SetTimestamp(input_frame.timestamp());
  encoded_image_.ntp_time_ms_ = input_frame.ntp_time_ms();
  encoded_image_.capture_time_ms_ = input_frame.render_time_ms();
  encoded_image_.rotation_ = input_frame.rotation();
  return DrainEncodedData();
}

int32_t ILH264Encoder::DrainEncodedData()
{
  OMX_ERRORTYPE err;
  OMX_BUFFERHEADERTYPE *out;
  out = ilclient_get_output_buffer(video_encode_, 201, 1);

  err = OMX_FillThisBuffer(ILC_GET_HANDLE(video_encode_), out);
  if (err != OMX_ErrorNone)
  {
    drop_next_frame_ = true;
  }

  fill_buffer_event_.Wait(rtc::Event::kForever);

  if (out != NULL && out->nFilledLen > 0)
  {
    if (out->nFlags & OMX_BUFFERFLAG_CODECCONFIG)
    {
      size_t num_nals = 0;
      const uint8_t *data = out->pBuffer + out->nOffset;
      size_t size = out->nFilledLen;

#if H264HWENC_HEADER_DEBUG
      for (uint32_t i = 0; i < out->nFilledLen; i++)
        printf("%02x ", *(data + i));
      printf("\n");
#endif

      const uint8_t *nalStart = nullptr;
      size_t nalSize = 0;
      while (getNextNALUnit(&data, &size, &nalStart, &nalSize, true))
      {
        if ((*nalStart & 0x1f) == kNALTypeSPS)
        {
          sps_length_ = nalSize + sizeof(kNALStartCode);
          memcpy(lastSPS_,
                 nalStart - sizeof(kNALStartCode),
                 sps_length_);
          lastSPS_[4] |= 0x40;
        }
        else if ((*nalStart & 0x1f) == kNALTypePPS)
        {
          pps_length_ = nalSize + sizeof(kNALStartCode);
          memcpy(lastPPS_,
                 nalStart - sizeof(kNALStartCode),
                 pps_length_);
          lastPPS_[4] |= 0x40;
        }
        num_nals++;
      }
      out->nFilledLen = 0;
      return DrainEncodedData();
    }
    else
    {
      size_t required_size = 0;
      size_t writtenSize = 0;
      uint64_t buf_time = (((((uint64_t)out->nTimeStamp.nHighPart << 32) & 0xFFFFFFFF00000000) + (uint64_t)out->nTimeStamp.nLowPart)) / 1000ll;
      if (buf_time != encoded_image_.Timestamp())
      {
        RTC_LOG(LS_ERROR) << "Error timestamp is not match. timestamp:" << encoded_image_.Timestamp() << " buf_time:" << buf_time;
      }
      if (buf_time < encoded_image_.Timestamp())
      {
        RTC_LOG(LS_ERROR) << "buf_time is yanger than timestamp. retry get buffer.";
        out->nFilledLen = 0;
        return DrainEncodedData();
      }

      if (out->nFlags & OMX_BUFFERFLAG_SYNCFRAME)
      {
        encoded_image_._frameType = webrtc::kVideoFrameKey;
        required_size = sps_length_ + pps_length_ + out->nFilledLen;
      }
      else
      {
        encoded_image_._frameType = webrtc::kVideoFrameDelta;
        required_size = out->nFilledLen;
      }

      if (encoded_image_.capacity() < required_size)
      {
        size_t new_capacity = CalcBufferSize(
            webrtc::VideoType::kI420, encoded_image_._encodedWidth, encoded_image_._encodedHeight);
        if (new_capacity < required_size)
        {
          new_capacity = required_size;
        }
        encoded_image_buffer_.reset(new uint8_t[new_capacity]);
        encoded_image_.set_buffer(encoded_image_buffer_.get(), new_capacity);
      }

      if (out->nFlags & OMX_BUFFERFLAG_SYNCFRAME)
      {
        memcpy(encoded_image_.data(),
               lastSPS_,
               sps_length_);
        writtenSize += sps_length_;
        memcpy(encoded_image_.data() + writtenSize,
               lastPPS_,
               pps_length_);
        writtenSize += pps_length_;
      }
      out->pBuffer[out->nOffset + 4] |= 0x40;
      memcpy(encoded_image_.data() + writtenSize,
             out->pBuffer + out->nOffset,
             out->nFilledLen);
      encoded_image_.set_size(writtenSize + out->nFilledLen);

      SendEncodedDataToCallback(encoded_image_);
      out->nFilledLen = 0;
    }
  }
  return WEBRTC_VIDEO_CODEC_OK;
}

void ILH264Encoder::SendEncodedDataToCallback(webrtc::EncodedImage encoded_image)
{
  struct nal_entry
  {
    uint32_t offset;
    uint32_t size;
  };
  std::vector<nal_entry> nals;

  const uint8_t *data = encoded_image.data();
  size_t size = encoded_image.size();
  const uint8_t *nalStart = nullptr;
  size_t nalSize = 0;
  while (getNextNALUnit(&data, &size, &nalStart, &nalSize, true))
  {
    nal_entry nal = {((uint32_t)(nalStart - encoded_image.data())), (uint32_t)nalSize};
    nals.push_back(nal);
  }

  size_t num_nals = nals.size();
  webrtc::RTPFragmentationHeader frag_header;
  frag_header.VerifyAndAllocateFragmentationHeader(num_nals);
  for (size_t i = 0; i < num_nals; i++)
  {
    frag_header.fragmentationOffset[i] = nals[i].offset;
    frag_header.fragmentationLength[i] = nals[i].size;
  }

  webrtc::CodecSpecificInfo codec_specific;
  codec_specific.codecType = webrtc::kVideoCodecH264;
  codec_specific.codecSpecific.H264.packetization_mode = packetization_mode_;

  h264_bitstream_parser_.ParseBitstream(data, size);
  h264_bitstream_parser_.GetLastSliceQp(&encoded_image.qp_);

  if (callback_->OnEncodedImage(encoded_image, &codec_specific, &frag_header).error != webrtc::EncodedImageCallback::Result::OK)
  {
    return;
  }
  bitrate_adjuster_.Update(size);
}

bool ILH264Encoder::IsInitialized() const
{
  return ilclient_ != nullptr;
}
