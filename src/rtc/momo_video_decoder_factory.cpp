#include "momo_video_decoder_factory.h"

// WebRTC
#include <absl/strings/match.h>
#include <api/video_codecs/sdp_video_format.h>
#include <media/base/codec.h>
#include <media/base/media_constants.h>
#include <modules/video_coding/codecs/h264/include/h264.h>
#include <modules/video_coding/codecs/vp8/include/vp8.h>
#include <modules/video_coding/codecs/vp9/include/vp9.h>
#include <rtc_base/checks.h>
#include <rtc_base/logging.h>

#if !defined(__arm__) || defined(__aarch64__) || defined(__ARM_NEON__)
#include <modules/video_coding/codecs/av1/libaom_av1_decoder.h>
#endif

#if defined(__APPLE__)
#include "mac_helper/objc_codec_factory_helper.h"
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

MomoVideoDecoderFactory::MomoVideoDecoderFactory(
    VideoCodecInfo::Type vp8_decoder,
    VideoCodecInfo::Type vp9_decoder,
    VideoCodecInfo::Type av1_decoder,
    VideoCodecInfo::Type h264_decoder)
    : vp8_decoder_(vp8_decoder),
      vp9_decoder_(vp9_decoder),
      av1_decoder_(av1_decoder),
      h264_decoder_(h264_decoder) {
#if defined(__APPLE__)
  video_decoder_factory_ = CreateObjCDecoderFactory();
#endif
}
std::vector<webrtc::SdpVideoFormat>
MomoVideoDecoderFactory::GetSupportedFormats() const {
  std::vector<webrtc::SdpVideoFormat> supported_codecs;

  // VP8
  if (vp8_decoder_ == VideoCodecInfo::Type::Software ||
      vp9_decoder_ == VideoCodecInfo::Type::Jetson) {
    supported_codecs.push_back(webrtc::SdpVideoFormat(cricket::kVp8CodecName));
  }

  // VP9
  if (vp9_decoder_ == VideoCodecInfo::Type::Software ||
      vp9_decoder_ == VideoCodecInfo::Type::Jetson) {
    for (const webrtc::SdpVideoFormat& format : webrtc::SupportedVP9Codecs()) {
      supported_codecs.push_back(format);
    }
  }

  // AV1
  // 今のところ Software のみ
  if (av1_decoder_ == VideoCodecInfo::Type::Software) {
    supported_codecs.push_back(webrtc::SdpVideoFormat(cricket::kAv1CodecName));
  }

  // H264
  std::vector<webrtc::SdpVideoFormat> h264_codecs = {
      CreateH264Format(webrtc::H264::kProfileBaseline, webrtc::H264::kLevel3_1,
                       "1"),
      CreateH264Format(webrtc::H264::kProfileBaseline, webrtc::H264::kLevel3_1,
                       "0"),
      CreateH264Format(webrtc::H264::kProfileConstrainedBaseline,
                       webrtc::H264::kLevel3_1, "1"),
      CreateH264Format(webrtc::H264::kProfileConstrainedBaseline,
                       webrtc::H264::kLevel3_1, "0")};

  if (h264_decoder_ == VideoCodecInfo::Type::VideoToolbox) {
    // VideoToolbox の場合は video_decoder_factory_ から H264 を拾ってくる
    for (auto format : video_decoder_factory_->GetSupportedFormats()) {
      if (absl::EqualsIgnoreCase(format.name, cricket::kH264CodecName)) {
        supported_codecs.push_back(format);
      }
    }
  } else if (h264_decoder_ != VideoCodecInfo::Type::NotSupported) {
    // その他のデコーダの場合は手動で追加
    for (const webrtc::SdpVideoFormat& h264_format : h264_codecs) {
      supported_codecs.push_back(h264_format);
    }
  }

  return supported_codecs;
}

std::unique_ptr<webrtc::VideoDecoder>
MomoVideoDecoderFactory::CreateVideoDecoder(
    const webrtc::SdpVideoFormat& format) {
  if (!IsFormatSupported(GetSupportedFormats(), format)) {
    RTC_LOG(LS_ERROR) << "Trying to create decoder for unsupported format";
    return nullptr;
  }

  if (absl::EqualsIgnoreCase(format.name, cricket::kVp8CodecName)) {
#if USE_JETSON_ENCODER
    if (vp8_decoder_ == VideoCodecInfo::Type::Jetson) {
      return std::unique_ptr<webrtc::VideoDecoder>(
          absl::make_unique<JetsonVideoDecoder>(V4L2_PIX_FMT_VP8));
    }
#endif

    if (vp8_decoder_ == VideoCodecInfo::Type::Software) {
      return webrtc::VP8Decoder::Create();
    }
  }

  if (absl::EqualsIgnoreCase(format.name, cricket::kVp9CodecName)) {
#if USE_JETSON_ENCODER
    if (vp9_decoder_ == VideoCodecInfo::Type::Jetson) {
      return std::unique_ptr<webrtc::VideoDecoder>(
          absl::make_unique<JetsonVideoDecoder>(V4L2_PIX_FMT_VP9));
    }
#endif

    if (vp9_decoder_ == VideoCodecInfo::Type::Software) {
      return webrtc::VP9Decoder::Create();
    }
  }

  if (absl::EqualsIgnoreCase(format.name, cricket::kAv1CodecName)) {
#if !defined(__arm__) || defined(__aarch64__) || defined(__ARM_NEON__)
    if (av1_decoder_ == VideoCodecInfo::Type::Software) {
      return webrtc::CreateLibaomAv1Decoder();
    }
#endif
  }

  if (absl::EqualsIgnoreCase(format.name, cricket::kH264CodecName)) {
#if USE_JETSON_ENCODER
    if (h264_decoder_ == VideoCodecInfo::Type::Jetson) {
      return std::unique_ptr<webrtc::VideoDecoder>(
          absl::make_unique<JetsonVideoDecoder>(V4L2_PIX_FMT_H264));
    }
#endif

#if USE_MMAL_ENCODER
    if (h264_decoder_ == VideoCodecInfo::Type::MMAL) {
      return std::unique_ptr<webrtc::VideoDecoder>(
          absl::make_unique<MMALH264Decoder>());
    }
#endif
  }

  RTC_NOTREACHED();
  return nullptr;
}
