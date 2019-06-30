#include "hw_video_encoder_factory.h"

#include "absl/strings/match.h"
#include "absl/memory/memory.h"
#include "api/video_codecs/sdp_video_format.h"
#include "media/base/codec.h"
#include "media/base/media_constants.h"
#include "modules/video_coding/codecs/h264/include/h264.h"
#include "modules/video_coding/codecs/vp8/include/vp8.h"
#include "modules/video_coding/codecs/vp9/include/vp9.h"
#include "rtc_base/logging.h"

#if USE_IL_ENCODER
#include "hwenc_il/il_h264_encoder.h"
#endif

std::vector<webrtc::SdpVideoFormat> HWVideoEncoderFactory::GetSupportedFormats()
    const {
  std::vector<webrtc::SdpVideoFormat> supported_codecs;
  supported_codecs.push_back(webrtc::SdpVideoFormat(cricket::kVp8CodecName));
  for (const webrtc::SdpVideoFormat& format : webrtc::SupportedVP9Codecs())
    supported_codecs.push_back(format);
  for (const webrtc::SdpVideoFormat& format : webrtc::SupportedH264Codecs())
    supported_codecs.push_back(format);

  return supported_codecs;
}

webrtc::VideoEncoderFactory::CodecInfo HWVideoEncoderFactory::QueryVideoEncoder(
    const webrtc::SdpVideoFormat& format) const {
  CodecInfo info;
  info.has_internal_source = false;
  if (absl::EqualsIgnoreCase(format.name, cricket::kH264CodecName))
    info.is_hardware_accelerated = true;
  else
    info.is_hardware_accelerated = false;
  return info;
}

std::unique_ptr<webrtc::VideoEncoder> HWVideoEncoderFactory::CreateVideoEncoder(
    const webrtc::SdpVideoFormat& format) {
  if (absl::EqualsIgnoreCase(format.name, cricket::kVp8CodecName))
    return webrtc::VP8Encoder::Create();

  if (absl::EqualsIgnoreCase(format.name, cricket::kVp9CodecName))
    return webrtc::VP9Encoder::Create(cricket::VideoCodec(format));

  if (absl::EqualsIgnoreCase(format.name, cricket::kH264CodecName)) {
#if USE_IL_ENCODER
    return std::unique_ptr<webrtc::VideoEncoder>(absl::make_unique<ILH264Encoder>(cricket::VideoCodec(format)));
#endif
  }

  RTC_LOG(LS_ERROR) << "Trying to created encoder of unsupported format "
                    << format.name;
  return nullptr;
}