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
#include <modules/video_coding/codecs/av1/av1_svc_config.h>
#include <modules/video_coding/codecs/av1/dav1d_decoder.h>
#endif

#if defined(__APPLE__)
#include "mac_helper/objc_codec_factory_helper.h"
#endif

#if defined(USE_JETSON_ENCODER)
#include "sora/hwenc_jetson/jetson_video_decoder.h"
#endif

#if defined(USE_NVCODEC_ENCODER)
#include "sora/hwenc_nvcodec/nvcodec_video_decoder.h"
#endif

#if defined(USE_VPL_ENCODER)
#include "sora/hwenc_vpl/vpl_video_decoder.h"
#endif

#if defined(USE_V4L2_ENCODER)
#include "hwenc_v4l2/v4l2_h264_decoder.h"
#endif

namespace {

bool IsFormatSupported(
    const std::vector<webrtc::SdpVideoFormat>& supported_formats,
    const webrtc::SdpVideoFormat& format) {
  for (const webrtc::SdpVideoFormat& supported_format : supported_formats) {
    if (format.IsSameCodec(supported_format)) {
      return true;
    }
  }
  return false;
}

}  // namespace

MomoVideoDecoderFactory::MomoVideoDecoderFactory(
    const MomoVideoDecoderFactoryConfig& config)
    : config_(config) {
#if defined(__APPLE__)
  video_decoder_factory_ = CreateObjCDecoderFactory();
#endif
}

std::vector<webrtc::SdpVideoFormat>
MomoVideoDecoderFactory::GetSupportedFormats() const {
  std::vector<webrtc::SdpVideoFormat> supported_codecs;

  auto add_vp8 = [&supported_codecs]() {
    supported_codecs.push_back(webrtc::SdpVideoFormat(webrtc::kVp8CodecName));
  };
  auto add_vp9 = [&supported_codecs]() {
    for (const webrtc::SdpVideoFormat& format :
         webrtc::SupportedVP9Codecs(true)) {
      supported_codecs.push_back(format);
    }
  };
  auto add_av1 = [&supported_codecs]() {
    supported_codecs.push_back(webrtc::SdpVideoFormat(
        webrtc::kAv1CodecName, webrtc::CodecParameterMap(),
        webrtc::LibaomAv1EncoderSupportedScalabilityModes()));
  };
  auto add_h264 = [&supported_codecs]() {
    std::vector<webrtc::SdpVideoFormat> h264_codecs = {
        CreateH264Format(webrtc::H264Profile::kProfileBaseline,
                         webrtc::H264Level::kLevel3_1, "1"),
        CreateH264Format(webrtc::H264Profile::kProfileBaseline,
                         webrtc::H264Level::kLevel3_1, "0"),
        CreateH264Format(webrtc::H264Profile::kProfileConstrainedBaseline,
                         webrtc::H264Level::kLevel3_1, "1"),
        CreateH264Format(webrtc::H264Profile::kProfileConstrainedBaseline,
                         webrtc::H264Level::kLevel3_1, "0")};

    for (const webrtc::SdpVideoFormat& format : h264_codecs) {
      supported_codecs.push_back(format);
    }
  };
  auto add_h265 = [&supported_codecs]() {
    supported_codecs.push_back(webrtc::SdpVideoFormat(webrtc::kH265CodecName));
  };

#if defined(USE_NVCODEC_ENCODER)
  if (config_.vp8_decoder == VideoCodecInfo::Type::NVIDIA &&
      sora::NvCodecVideoDecoder::IsSupported(config_.cuda_context,
                                             sora::CudaVideoCodec::VP8)) {
    add_vp8();
  }
  if (config_.vp9_decoder == VideoCodecInfo::Type::NVIDIA &&
      sora::NvCodecVideoDecoder::IsSupported(config_.cuda_context,
                                             sora::CudaVideoCodec::VP9)) {
    add_vp9();
  }
  if (config_.av1_decoder == VideoCodecInfo::Type::NVIDIA &&
      sora::NvCodecVideoDecoder::IsSupported(config_.cuda_context,
                                             sora::CudaVideoCodec::AV1)) {
    add_av1();
  }
  if (config_.h264_decoder == VideoCodecInfo::Type::NVIDIA &&
      sora::NvCodecVideoDecoder::IsSupported(config_.cuda_context,
                                             sora::CudaVideoCodec::H264)) {
    add_h264();
  }
  if (config_.h265_decoder == VideoCodecInfo::Type::NVIDIA &&
      sora::NvCodecVideoDecoder::IsSupported(config_.cuda_context,
                                             sora::CudaVideoCodec::H265)) {
    add_h265();
  }
#endif

#if defined(USE_VPL_ENCODER)
  auto session = sora::VplSession::Create();
  if (config_.vp8_decoder == VideoCodecInfo::Type::Intel &&
      sora::VplVideoDecoder::IsSupported(session, webrtc::kVideoCodecVP8)) {
    add_vp8();
  }
  if (config_.vp9_decoder == VideoCodecInfo::Type::Intel &&
      sora::VplVideoDecoder::IsSupported(session, webrtc::kVideoCodecVP9)) {
    add_vp9();
  }
  if (config_.av1_decoder == VideoCodecInfo::Type::Intel &&
      sora::VplVideoDecoder::IsSupported(session, webrtc::kVideoCodecAV1)) {
    add_av1();
  }
  if (config_.h264_decoder == VideoCodecInfo::Type::Intel &&
      sora::VplVideoDecoder::IsSupported(session, webrtc::kVideoCodecH264)) {
    add_h264();
  }
  if (config_.h265_decoder == VideoCodecInfo::Type::Intel &&
      sora::VplVideoDecoder::IsSupported(session, webrtc::kVideoCodecH265)) {
    add_h265();
  }
#endif

#if defined(USE_V4L2_ENCODER)
  if (config_.h264_decoder == VideoCodecInfo::Type::V4L2) {
    add_h264();
  }
#endif

#if defined(USE_JETSON_ENCODER)
  if (config_.vp8_decoder == VideoCodecInfo::Type::Jetson &&
      sora::JetsonVideoDecoder::IsSupported(webrtc::kVideoCodecVP8)) {
    add_vp8();
  }
  if (config_.vp9_decoder == VideoCodecInfo::Type::Jetson &&
      sora::JetsonVideoDecoder::IsSupported(webrtc::kVideoCodecVP9)) {
    add_vp9();
  }
  if (config_.av1_decoder == VideoCodecInfo::Type::Jetson &&
      sora::JetsonVideoDecoder::IsSupported(webrtc::kVideoCodecAV1)) {
    add_av1();
  }
  if (config_.h264_decoder == VideoCodecInfo::Type::Jetson &&
      sora::JetsonVideoDecoder::IsSupported(webrtc::kVideoCodecH264)) {
    add_h264();
  }
  if (config_.h265_decoder == VideoCodecInfo::Type::Jetson &&
      sora::JetsonVideoDecoder::IsSupported(webrtc::kVideoCodecH265)) {
    add_h265();
  }
#endif

#if defined(__APPLE__)
  // VideoToolbox の場合は video_decoder_factory_ から拾ってくる
  auto formats = video_decoder_factory_->GetSupportedFormats();
  if (config_.vp8_decoder == VideoCodecInfo::Type::VideoToolbox) {
    for (auto format : formats) {
      if (absl::EqualsIgnoreCase(format.name, webrtc::kVp8CodecName)) {
        supported_codecs.push_back(format);
      }
    }
  }
  if (config_.vp9_decoder == VideoCodecInfo::Type::VideoToolbox) {
    for (auto format : formats) {
      if (absl::EqualsIgnoreCase(format.name, webrtc::kVp9CodecName)) {
        supported_codecs.push_back(format);
      }
    }
  }
  if (config_.av1_decoder == VideoCodecInfo::Type::VideoToolbox) {
    for (auto format : formats) {
      if (absl::EqualsIgnoreCase(format.name, webrtc::kAv1CodecName)) {
        supported_codecs.push_back(format);
      }
    }
  }
  if (config_.h264_decoder == VideoCodecInfo::Type::VideoToolbox) {
    for (auto format : formats) {
      if (absl::EqualsIgnoreCase(format.name, webrtc::kH264CodecName)) {
        supported_codecs.push_back(format);
      }
    }
  }
  if (config_.h265_decoder == VideoCodecInfo::Type::VideoToolbox) {
    for (auto format : formats) {
      if (absl::EqualsIgnoreCase(format.name, webrtc::kH265CodecName)) {
        supported_codecs.push_back(format);
      }
    }
  }
#endif

  if (config_.vp8_decoder == VideoCodecInfo::Type::Software) {
    add_vp8();
  }
  if (config_.vp9_decoder == VideoCodecInfo::Type::Software) {
    add_vp9();
  }
  if (config_.av1_decoder == VideoCodecInfo::Type::Software) {
    add_av1();
  }
  if (config_.h264_decoder == VideoCodecInfo::Type::Software) {
    add_h264();
  }
  if (config_.h265_decoder == VideoCodecInfo::Type::Software) {
    add_h265();
  }

  return supported_codecs;
}

std::unique_ptr<webrtc::VideoDecoder> MomoVideoDecoderFactory::Create(
    const webrtc::Environment& env,
    const webrtc::SdpVideoFormat& format) {
  if (!IsFormatSupported(GetSupportedFormats(), format)) {
    RTC_LOG(LS_ERROR) << "Trying to create decoder for unsupported format";
    return nullptr;
  }

  auto is_vp8 = absl::EqualsIgnoreCase(format.name, webrtc::kVp8CodecName);
  auto is_vp9 = absl::EqualsIgnoreCase(format.name, webrtc::kVp9CodecName);
  auto is_av1 = absl::EqualsIgnoreCase(format.name, webrtc::kAv1CodecName);
  auto is_h264 = absl::EqualsIgnoreCase(format.name, webrtc::kH264CodecName);
  auto is_h265 = absl::EqualsIgnoreCase(format.name, webrtc::kH265CodecName);

#if defined(USE_NVCODEC_ENCODER)
  if (is_vp8 && config_.vp8_decoder == VideoCodecInfo::Type::NVIDIA) {
    return std::make_unique<sora::NvCodecVideoDecoder>(
        config_.cuda_context, sora::CudaVideoCodec::VP8);
  }
  if (is_vp9 && config_.vp9_decoder == VideoCodecInfo::Type::NVIDIA) {
    return std::make_unique<sora::NvCodecVideoDecoder>(
        config_.cuda_context, sora::CudaVideoCodec::VP9);
  }
  if (is_av1 && config_.av1_decoder == VideoCodecInfo::Type::NVIDIA) {
    return std::make_unique<sora::NvCodecVideoDecoder>(
        config_.cuda_context, sora::CudaVideoCodec::AV1);
  }
  if (is_h264 && config_.h264_decoder == VideoCodecInfo::Type::NVIDIA) {
    return std::make_unique<sora::NvCodecVideoDecoder>(
        config_.cuda_context, sora::CudaVideoCodec::H264);
  }
  if (is_h265 && config_.h265_decoder == VideoCodecInfo::Type::NVIDIA) {
    return std::make_unique<sora::NvCodecVideoDecoder>(
        config_.cuda_context, sora::CudaVideoCodec::H265);
  }
#endif

#if defined(USE_VPL_ENCODER)
  auto session = sora::VplSession::Create();
  if (is_vp8 && config_.vp8_decoder == VideoCodecInfo::Type::Intel) {
    return sora::VplVideoDecoder::Create(session, webrtc::kVideoCodecVP8);
  }
  if (is_vp9 && config_.vp9_decoder == VideoCodecInfo::Type::Intel) {
    return sora::VplVideoDecoder::Create(session, webrtc::kVideoCodecVP9);
  }
  if (is_av1 && config_.av1_decoder == VideoCodecInfo::Type::Intel) {
    return sora::VplVideoDecoder::Create(session, webrtc::kVideoCodecAV1);
  }
  if (is_h264 && config_.h264_decoder == VideoCodecInfo::Type::Intel) {
    return sora::VplVideoDecoder::Create(session, webrtc::kVideoCodecH264);
  }
  if (is_h265 && config_.h265_decoder == VideoCodecInfo::Type::Intel) {
    return sora::VplVideoDecoder::Create(session, webrtc::kVideoCodecH265);
  }
#endif

#if defined(USE_V4L2_ENCODER)
  if (is_h264 && config_.h264_decoder == VideoCodecInfo::Type::V4L2) {
    return std::unique_ptr<webrtc::VideoDecoder>(
        std::make_unique<V4L2H264Decoder>(webrtc::kVideoCodecH264));
  }
#endif

#if defined(USE_JETSON_ENCODER)
  if (is_vp8 && config_.vp8_decoder == VideoCodecInfo::Type::Jetson) {
    return std::unique_ptr<webrtc::VideoDecoder>(
        std::make_unique<sora::JetsonVideoDecoder>(webrtc::kVideoCodecVP8));
  }
  if (is_vp9 && config_.vp9_decoder == VideoCodecInfo::Type::Jetson) {
    return std::unique_ptr<webrtc::VideoDecoder>(
        std::make_unique<sora::JetsonVideoDecoder>(webrtc::kVideoCodecVP9));
  }
  if (is_av1 && config_.av1_decoder == VideoCodecInfo::Type::Jetson) {
    return std::unique_ptr<webrtc::VideoDecoder>(
        std::make_unique<sora::JetsonVideoDecoder>(webrtc::kVideoCodecAV1));
  }
  if (is_h264 && config_.h264_decoder == VideoCodecInfo::Type::Jetson) {
    return std::unique_ptr<webrtc::VideoDecoder>(
        std::make_unique<sora::JetsonVideoDecoder>(webrtc::kVideoCodecH264));
  }
  if (is_h265 && config_.h265_decoder == VideoCodecInfo::Type::Jetson) {
    return std::unique_ptr<webrtc::VideoDecoder>(
        std::make_unique<sora::JetsonVideoDecoder>(webrtc::kVideoCodecH265));
  }
#endif

#if defined(__APPLE__)
  if (is_vp8 && config_.vp8_decoder == VideoCodecInfo::Type::VideoToolbox) {
    return video_decoder_factory_->Create(env, format);
  }
  if (is_vp9 && config_.vp9_decoder == VideoCodecInfo::Type::VideoToolbox) {
    return video_decoder_factory_->Create(env, format);
  }
  if (is_av1 && config_.av1_decoder == VideoCodecInfo::Type::VideoToolbox) {
    return video_decoder_factory_->Create(env, format);
  }
  if (is_h264 && config_.h264_decoder == VideoCodecInfo::Type::VideoToolbox) {
    return video_decoder_factory_->Create(env, format);
  }
  if (is_h265 && config_.h265_decoder == VideoCodecInfo::Type::VideoToolbox) {
    return video_decoder_factory_->Create(env, format);
  }
#endif

  if (is_vp8 && config_.vp8_decoder == VideoCodecInfo::Type::Software) {
    return webrtc::CreateVp8Decoder(env);
  }
  if (is_vp9 && config_.vp9_decoder == VideoCodecInfo::Type::Software) {
    return webrtc::VP9Decoder::Create();
  }
  if (is_av1 && config_.av1_decoder == VideoCodecInfo::Type::Software) {
    return webrtc::CreateDav1dDecoder();
  }
  if (is_h264 && config_.h264_decoder == VideoCodecInfo::Type::Software) {
    return nullptr;
  }
  if (is_h265 && config_.h265_decoder == VideoCodecInfo::Type::Software) {
    return nullptr;
  }

  RTC_DCHECK_NOTREACHED();
  return nullptr;
}
