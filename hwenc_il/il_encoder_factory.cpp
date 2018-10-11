#include "il_encoder_factory.h"

#include <utility>

#include "modules/video_coding/codecs/h264/include/h264.h"
#include "modules/video_coding/codecs/vp9/include/vp9.h"
#include "media/engine/vp8_encoder_simulcast_proxy.h"
#include "rtc_base/logging.h"
#include "absl/memory/memory.h"

#include "il_h264_encoder.h"

std::vector<webrtc::SdpVideoFormat> ILVideoEncoderFactory::GetSupportedFormats()
    const {
  std::vector<webrtc::SdpVideoFormat> supported_codecs;
  supported_codecs.push_back(webrtc::SdpVideoFormat(cricket::kVp8CodecName));
  for (const webrtc::SdpVideoFormat& format : webrtc::SupportedVP9Codecs())
    supported_codecs.push_back(format);
  for (const webrtc::SdpVideoFormat& format : webrtc::SupportedH264Codecs())
    supported_codecs.push_back(format);

  return supported_codecs;
}

webrtc::VideoEncoderFactory::CodecInfo ILVideoEncoderFactory::QueryVideoEncoder(
    const webrtc::SdpVideoFormat& format) const {
  CodecInfo info;
  info.has_internal_source = false;
  if (cricket::CodecNamesEq(format.name, cricket::kH264CodecName))
    info.is_hardware_accelerated = true;
  else
    info.is_hardware_accelerated = false;
  return info;
}

std::unique_ptr<webrtc::VideoEncoder> ILVideoEncoderFactory::CreateVideoEncoder(
    const webrtc::SdpVideoFormat& format) {
  if (cricket::CodecNamesEq(format.name, cricket::kVp8CodecName))
    return webrtc::VP8Encoder::Create();

  if (cricket::CodecNamesEq(format.name, cricket::kVp9CodecName))
    return webrtc::VP9Encoder::Create();

  if (cricket::CodecNamesEq(format.name, cricket::kH264CodecName))
    return std::unique_ptr<webrtc::VideoEncoder>(absl::make_unique<ILH264Encoder>(cricket::VideoCodec(format)));

  RTC_LOG(LS_ERROR) << "Trying to created encoder of unsupported format "
                    << format.name;
  return nullptr;
}
