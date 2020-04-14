/*
 *  Copyright (c) 2016 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "hw_video_decoder_factory.h"

#include "absl/strings/match.h"
#include "api/video_codecs/sdp_video_format.h"
#include "media/base/codec.h"
#include "media/base/media_constants.h"
#include "modules/video_coding/codecs/h264/include/h264.h"
#include "modules/video_coding/codecs/vp8/include/vp8.h"
#include "modules/video_coding/codecs/vp9/include/vp9.h"
#include "rtc_base/checks.h"
#include "rtc_base/logging.h"

#if !defined(__arm__) || defined(__aarch64__) || defined(__ARM_NEON__)
#include "modules/video_coding/codecs/av1/libaom_av1_decoder.h"
#endif

#if USE_JETSON_ENCODER
#include "hwenc_jetson/jetson_video_decoder.h"
#elif USE_MMAL_ENCODER
#include "hwenc_mmal/mmal_h264_decoder.h"
#endif

#include "h264_format.h"

namespace {

bool IsFormatSupported(
    const std::vector<webrtc::SdpVideoFormat>& supported_formats,
    const webrtc::SdpVideoFormat& format) {
  for (const webrtc::SdpVideoFormat& supported_format : supported_formats) {
    if (cricket::IsSameCodec(format.name, format.parameters,
                             supported_format.name,
                             supported_format.parameters)) {
      return true;
    }
  }
  return false;
}

}  // namespace

std::vector<webrtc::SdpVideoFormat> HWVideoDecoderFactory::GetSupportedFormats()
    const {
  std::vector<webrtc::SdpVideoFormat> formats;
  formats.push_back(webrtc::SdpVideoFormat(cricket::kVp8CodecName));
  for (const webrtc::SdpVideoFormat& format : webrtc::SupportedVP9Codecs())
    formats.push_back(format);

  std::vector<webrtc::SdpVideoFormat> h264_codecs = {
      CreateH264Format(webrtc::H264::kProfileBaseline, webrtc::H264::kLevel3_1,
                       "1"),
      CreateH264Format(webrtc::H264::kProfileBaseline, webrtc::H264::kLevel3_1,
                       "0"),
      CreateH264Format(webrtc::H264::kProfileConstrainedBaseline,
                       webrtc::H264::kLevel3_1, "1"),
      CreateH264Format(webrtc::H264::kProfileConstrainedBaseline,
                       webrtc::H264::kLevel3_1, "0")};

  for (const webrtc::SdpVideoFormat& h264_format : h264_codecs)
    formats.push_back(h264_format);

#if !defined(__arm__) || defined(__aarch64__) || defined(__ARM_NEON__)
  formats.push_back(webrtc::SdpVideoFormat(cricket::kAv1CodecName));
#endif

  return formats;
}

std::unique_ptr<webrtc::VideoDecoder> HWVideoDecoderFactory::CreateVideoDecoder(
    const webrtc::SdpVideoFormat& format) {
  if (!IsFormatSupported(GetSupportedFormats(), format)) {
    RTC_LOG(LS_ERROR) << "Trying to create decoder for unsupported format";
    return nullptr;
  }

#if USE_JETSON_ENCODER
  uint32_t input_format = 0;
  if (absl::EqualsIgnoreCase(format.name, cricket::kVp8CodecName))
    input_format = V4L2_PIX_FMT_VP8;
  if (absl::EqualsIgnoreCase(format.name, cricket::kVp9CodecName))
    input_format = V4L2_PIX_FMT_VP9;
  if (absl::EqualsIgnoreCase(format.name, cricket::kH264CodecName))
    input_format = V4L2_PIX_FMT_H264;
  if (input_format != 0) {
    return std::unique_ptr<webrtc::VideoDecoder>(
        absl::make_unique<JetsonVideoDecoder>(input_format));
  }
#else
  if (absl::EqualsIgnoreCase(format.name, cricket::kVp8CodecName))
    return webrtc::VP8Decoder::Create();
  if (absl::EqualsIgnoreCase(format.name, cricket::kVp9CodecName))
    return webrtc::VP9Decoder::Create();
#if USE_MMAL_ENCODER
  if (absl::EqualsIgnoreCase(format.name, cricket::kH264CodecName))
    return std::unique_ptr<webrtc::VideoDecoder>(
        absl::make_unique<MMALH264Decoder>());
#endif
#endif

#if !defined(__arm__) || defined(__aarch64__) || defined(__ARM_NEON__)
  if (absl::EqualsIgnoreCase(format.name, cricket::kAv1CodecName))
    return webrtc::CreateLibaomAv1Decoder();
#endif

  RTC_NOTREACHED();
  return nullptr;
}
