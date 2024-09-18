#include "momo_video_encoder_factory.h"

#include <iostream>

// WebRTC
#include <absl/memory/memory.h>
#include <absl/strings/match.h>
#include <api/environment/environment_factory.h>
#include <api/video_codecs/sdp_video_format.h>
#include <api/video_codecs/video_codec.h>
#include <api/video_codecs/vp9_profile.h>
#include <media/base/codec.h>
#include <media/base/media_constants.h>
#include <media/engine/simulcast_encoder_adapter.h>
#include <modules/video_coding/codecs/h264/include/h264.h>
#include <modules/video_coding/codecs/vp8/include/vp8.h>
#include <modules/video_coding/codecs/vp9/include/vp9.h>
#include <rtc_base/logging.h>

#if !defined(__arm__) || defined(__aarch64__) || defined(__ARM_NEON__)
#include <modules/video_coding/codecs/av1/av1_svc_config.h>
#include <modules/video_coding/codecs/av1/libaom_av1_encoder.h>
#endif

#if defined(__APPLE__)
#include "mac_helper/objc_codec_factory_helper.h"
#endif

#if defined(USE_JETSON_ENCODER)
#include "sora/hwenc_jetson/jetson_video_encoder.h"
#endif
#if defined(USE_NVCODEC_ENCODER)
#include "sora/hwenc_nvcodec/nvcodec_video_encoder.h"
#endif
#if defined(USE_VPL_ENCODER)
#include "sora/hwenc_vpl/vpl_video_encoder.h"
#endif
#if defined(USE_V4L2_ENCODER)
#include "hwenc_v4l2/v4l2_h264_encoder.h"
#endif

#include "sora/open_h264_video_encoder.h"

#include "rtc/aligned_encoder_adapter.h"

MomoVideoEncoderFactory::MomoVideoEncoderFactory(
    const MomoVideoEncoderFactoryConfig& config)
    : config_(config) {
#if defined(__APPLE__)
  video_encoder_factory_ = CreateObjCEncoderFactory();
#endif
  if (config.simulcast) {
    auto config2 = config;
    config2.simulcast = false;
    internal_encoder_factory_.reset(new MomoVideoEncoderFactory(config2));
  }
}

std::vector<webrtc::SdpVideoFormat>
MomoVideoEncoderFactory::GetSupportedFormats() const {
  std::vector<webrtc::SdpVideoFormat> supported_codecs;

  auto add_vp8 = [&supported_codecs]() {
    supported_codecs.push_back(webrtc::SdpVideoFormat(cricket::kVp8CodecName));
  };
  auto add_vp9 = [&supported_codecs]() {
    for (const webrtc::SdpVideoFormat& format :
         webrtc::SupportedVP9Codecs(true)) {
      supported_codecs.push_back(format);
    }
  };
  auto add_av1 = [&supported_codecs]() {
    supported_codecs.push_back(webrtc::SdpVideoFormat(
        cricket::kAv1CodecName, webrtc::SdpVideoFormat::Parameters(),
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
    supported_codecs.push_back(webrtc::SdpVideoFormat(cricket::kH265CodecName));
  };

#if defined(USE_NVCODEC_ENCODER)
  if (config_.vp8_encoder == VideoCodecInfo::Type::NVIDIA &&
      sora::NvCodecVideoEncoder::IsSupported(config_.cuda_context,
                                             sora::CudaVideoCodec::VP8)) {
    add_vp8();
  }
  if (config_.vp9_encoder == VideoCodecInfo::Type::NVIDIA &&
      sora::NvCodecVideoEncoder::IsSupported(config_.cuda_context,
                                             sora::CudaVideoCodec::VP9)) {
    add_vp9();
  }
  if (config_.av1_encoder == VideoCodecInfo::Type::NVIDIA &&
      sora::NvCodecVideoEncoder::IsSupported(config_.cuda_context,
                                             sora::CudaVideoCodec::AV1)) {
    add_av1();
  }
  if (config_.h264_encoder == VideoCodecInfo::Type::NVIDIA &&
      sora::NvCodecVideoEncoder::IsSupported(config_.cuda_context,
                                             sora::CudaVideoCodec::H264)) {
    add_h264();
  }
  if (config_.h265_encoder == VideoCodecInfo::Type::NVIDIA &&
      sora::NvCodecVideoEncoder::IsSupported(config_.cuda_context,
                                             sora::CudaVideoCodec::H265)) {
    add_h265();
  }
#endif

#if defined(USE_VPL_ENCODER)
  auto session = sora::VplSession::Create();
  if (config_.vp8_encoder == VideoCodecInfo::Type::Intel &&
      sora::VplVideoEncoder::IsSupported(session, webrtc::kVideoCodecVP8)) {
    add_vp8();
  }
  if (config_.vp9_encoder == VideoCodecInfo::Type::Intel &&
      sora::VplVideoEncoder::IsSupported(session, webrtc::kVideoCodecVP9)) {
    add_vp9();
  }
  if (config_.av1_encoder == VideoCodecInfo::Type::Intel &&
      sora::VplVideoEncoder::IsSupported(session, webrtc::kVideoCodecAV1)) {
    add_av1();
  }
  if (config_.h264_encoder == VideoCodecInfo::Type::Intel &&
      sora::VplVideoEncoder::IsSupported(session, webrtc::kVideoCodecH264)) {
    add_h264();
  }
  if (config_.h265_encoder == VideoCodecInfo::Type::Intel &&
      sora::VplVideoEncoder::IsSupported(session, webrtc::kVideoCodecH265)) {
    add_h265();
  }
#endif

#if defined(USE_V4L2_ENCODER)
  if (config_.h264_encoder == VideoCodecInfo::Type::V4L2) {
    add_h264();
  }
#endif

#if defined(USE_JETSON_ENCODER)
  if (config_.vp8_encoder == VideoCodecInfo::Type::Jetson &&
      sora::JetsonVideoEncoder::IsSupported(webrtc::kVideoCodecVP8)) {
    add_vp8();
  }
  if (config_.vp9_encoder == VideoCodecInfo::Type::Jetson &&
      sora::JetsonVideoEncoder::IsSupported(webrtc::kVideoCodecVP9)) {
    add_vp9();
  }
  if (config_.av1_encoder == VideoCodecInfo::Type::Jetson &&
      sora::JetsonVideoEncoder::IsSupported(webrtc::kVideoCodecAV1)) {
    add_av1();
  }
  if (config_.h264_encoder == VideoCodecInfo::Type::Jetson &&
      sora::JetsonVideoEncoder::IsSupported(webrtc::kVideoCodecH264)) {
    add_h264();
  }
  if (config_.h265_encoder == VideoCodecInfo::Type::Jetson &&
      sora::JetsonVideoEncoder::IsSupported(webrtc::kVideoCodecH265)) {
    add_h265();
  }
#endif

#if defined(__APPLE__)
  // VideoToolbox の場合は video_encoder_factory_ から拾ってくる
  auto formats = video_encoder_factory_->GetSupportedFormats();
  if (config_.vp8_encoder == VideoCodecInfo::Type::VideoToolbox) {
    for (auto format : formats) {
      if (absl::EqualsIgnoreCase(format.name, cricket::kVp8CodecName)) {
        supported_codecs.push_back(format);
      }
    }
  }
  if (config_.vp9_encoder == VideoCodecInfo::Type::VideoToolbox) {
    for (auto format : formats) {
      if (absl::EqualsIgnoreCase(format.name, cricket::kVp9CodecName)) {
        supported_codecs.push_back(format);
      }
    }
  }
  if (config_.av1_encoder == VideoCodecInfo::Type::VideoToolbox) {
    for (auto format : formats) {
      if (absl::EqualsIgnoreCase(format.name, cricket::kAv1CodecName)) {
        supported_codecs.push_back(format);
      }
    }
  }
  if (config_.h264_encoder == VideoCodecInfo::Type::VideoToolbox) {
    for (auto format : formats) {
      if (absl::EqualsIgnoreCase(format.name, cricket::kH264CodecName)) {
        supported_codecs.push_back(format);
      }
    }
  }
  if (config_.h265_encoder == VideoCodecInfo::Type::VideoToolbox) {
    for (auto format : formats) {
      if (absl::EqualsIgnoreCase(format.name, cricket::kH265CodecName)) {
        supported_codecs.push_back(format);
      }
    }
  }
#endif

  if (config_.vp8_encoder == VideoCodecInfo::Type::Software) {
    add_vp8();
  }
  if (config_.vp9_encoder == VideoCodecInfo::Type::Software) {
    add_vp9();
  }
  if (config_.av1_encoder == VideoCodecInfo::Type::Software) {
    add_av1();
  }
  if (config_.h264_encoder == VideoCodecInfo::Type::Software) {
    add_h264();
  }
  if (config_.h265_encoder == VideoCodecInfo::Type::Software) {
    add_h265();
  }

  return supported_codecs;
}

std::unique_ptr<webrtc::VideoEncoder> MomoVideoEncoderFactory::Create(
    const webrtc::Environment& env,
    const webrtc::SdpVideoFormat& format) {
  return WithSimulcast(format,
                       [this, &env](const webrtc::SdpVideoFormat& format) {
                         return CreateInternal(env, format);
                       });
}

std::unique_ptr<webrtc::VideoEncoder> MomoVideoEncoderFactory::CreateInternal(
    const webrtc::Environment& env,
    const webrtc::SdpVideoFormat& format) {
  auto is_vp8 = absl::EqualsIgnoreCase(format.name, cricket::kVp8CodecName);
  auto is_vp9 = absl::EqualsIgnoreCase(format.name, cricket::kVp9CodecName);
  auto is_av1 = absl::EqualsIgnoreCase(format.name, cricket::kAv1CodecName);
  auto is_h264 = absl::EqualsIgnoreCase(format.name, cricket::kH264CodecName);
  auto is_h265 = absl::EqualsIgnoreCase(format.name, cricket::kH265CodecName);

  // hardware_encoder_only == true の場合、Software なコーデックだったら強制終了する
  if (config_.hardware_encoder_only) {
    bool use_software = false;
    if (is_vp8 && config_.vp8_encoder == VideoCodecInfo::Type::Software) {
      use_software = true;
    }
    if (is_vp9 && config_.vp9_encoder == VideoCodecInfo::Type::Software) {
      use_software = true;
    }
    if (is_av1 && config_.av1_encoder == VideoCodecInfo::Type::Software) {
      use_software = true;
    }
    if (is_h264 && config_.h264_encoder == VideoCodecInfo::Type::Software) {
      use_software = true;
    }
    if (is_h265 && config_.h265_encoder == VideoCodecInfo::Type::Software) {
      use_software = true;
    }

    if (use_software) {
      std::cerr
          << "The software encoder is not available at the current setting."
          << std::endl;
      std::cerr << "Check the list of available encoders by specifying "
                   "--video-codec-engines."
                << std::endl;
      std::cerr
          << "To enable software encoders, specify --hw-mjpeg-decoder=false."
          << std::endl;
      std::exit(1);
    }
  }

#if defined(USE_NVCODEC_ENCODER)
  if (is_vp8 && config_.vp8_encoder == VideoCodecInfo::Type::NVIDIA) {
    return sora::NvCodecVideoEncoder::Create(config_.cuda_context,
                                             sora::CudaVideoCodec::VP8);
  }
  if (is_vp9 && config_.vp9_encoder == VideoCodecInfo::Type::NVIDIA) {
    return sora::NvCodecVideoEncoder::Create(config_.cuda_context,
                                             sora::CudaVideoCodec::VP9);
  }
  if (is_av1 && config_.av1_encoder == VideoCodecInfo::Type::NVIDIA) {
    return sora::NvCodecVideoEncoder::Create(config_.cuda_context,
                                             sora::CudaVideoCodec::AV1);
  }
  if (is_h264 && config_.h264_encoder == VideoCodecInfo::Type::NVIDIA) {
    return sora::NvCodecVideoEncoder::Create(config_.cuda_context,
                                             sora::CudaVideoCodec::H264);
  }
  if (is_h265 && config_.h265_encoder == VideoCodecInfo::Type::NVIDIA) {
    return sora::NvCodecVideoEncoder::Create(config_.cuda_context,
                                             sora::CudaVideoCodec::H265);
  }
#endif

#if defined(USE_VPL_ENCODER)
  auto session = sora::VplSession::Create();
  if (is_vp8 && config_.vp8_encoder == VideoCodecInfo::Type::Intel) {
    return sora::VplVideoEncoder::Create(session, webrtc::kVideoCodecVP8);
  }
  if (is_vp9 && config_.vp9_encoder == VideoCodecInfo::Type::Intel) {
    return sora::VplVideoEncoder::Create(session, webrtc::kVideoCodecVP9);
  }
  if (is_av1 && config_.av1_encoder == VideoCodecInfo::Type::Intel) {
    return sora::VplVideoEncoder::Create(session, webrtc::kVideoCodecAV1);
  }
  if (is_h264 && config_.h264_encoder == VideoCodecInfo::Type::Intel) {
    return sora::VplVideoEncoder::Create(session, webrtc::kVideoCodecH264);
  }
  if (is_h265 && config_.h265_encoder == VideoCodecInfo::Type::Intel) {
    return sora::VplVideoEncoder::Create(session, webrtc::kVideoCodecH265);
  }
#endif

#if defined(USE_V4L2_ENCODER)
  auto codec = cricket::CreateVideoCodec(format);
  if (is_h264 && config_.h264_encoder == VideoCodecInfo::Type::V4L2) {
    return std::make_unique<V4L2H264Encoder>(codec);
  }
#endif

#if defined(USE_JETSON_ENCODER)
  auto codec = cricket::CreateVideoCodec(format);
  if (is_vp8 && config_.vp8_encoder == VideoCodecInfo::Type::Jetson) {
    return std::make_unique<sora::JetsonVideoEncoder>(codec);
  }
  if (is_vp9 && config_.vp9_encoder == VideoCodecInfo::Type::Jetson) {
    return std::make_unique<sora::JetsonVideoEncoder>(codec);
  }
  if (is_av1 && config_.av1_encoder == VideoCodecInfo::Type::Jetson) {
    return std::make_unique<sora::JetsonVideoEncoder>(codec);
  }
  if (is_h264 && config_.h264_encoder == VideoCodecInfo::Type::Jetson) {
    return std::make_unique<sora::JetsonVideoEncoder>(codec);
  }
  if (is_h265 && config_.h265_encoder == VideoCodecInfo::Type::Jetson) {
    return std::make_unique<sora::JetsonVideoEncoder>(codec);
  }
#endif

#if defined(__APPLE__)
  if (is_vp8 && config_.vp8_encoder == VideoCodecInfo::Type::VideoToolbox) {
    return video_encoder_factory_->Create(env, format);
  }
  if (is_vp9 && config_.vp9_encoder == VideoCodecInfo::Type::VideoToolbox) {
    return video_encoder_factory_->Create(env, format);
  }
  if (is_av1 && config_.av1_encoder == VideoCodecInfo::Type::VideoToolbox) {
    return video_encoder_factory_->Create(env, format);
  }
  if (is_h264 && config_.h264_encoder == VideoCodecInfo::Type::VideoToolbox) {
    return video_encoder_factory_->Create(env, format);
  }
  if (is_h265 && config_.h265_encoder == VideoCodecInfo::Type::VideoToolbox) {
    return video_encoder_factory_->Create(env, format);
  }
#endif

  if (is_vp8 && config_.vp8_encoder == VideoCodecInfo::Type::Software) {
    return webrtc::CreateVp8Encoder(env);
  }
  if (is_vp9 && config_.vp9_encoder == VideoCodecInfo::Type::Software) {
    return webrtc::CreateVp9Encoder(env);
  }
  if (is_av1 && config_.av1_encoder == VideoCodecInfo::Type::Software) {
    return webrtc::CreateLibaomAv1Encoder(env);
  }
  if (is_h264 && config_.h264_encoder == VideoCodecInfo::Type::Software) {
    return sora::CreateOpenH264VideoEncoder(format, config_.openh264);
  }
  // if (is_h265 && config_.h265_encoder == VideoCodecInfo::Type::Software) {
  //   return nullptr;
  // }

  RTC_LOG(LS_ERROR) << "Trying to created encoder of unsupported format "
                    << format.name;
  return nullptr;
}

std::unique_ptr<webrtc::VideoEncoder> MomoVideoEncoderFactory::WithSimulcast(
    const webrtc::SdpVideoFormat& format,
    std::function<std::unique_ptr<webrtc::VideoEncoder>(
        const webrtc::SdpVideoFormat&)> create) {
  std::shared_ptr<webrtc::VideoEncoder> encoder;
  if (internal_encoder_factory_) {
    encoder = std::make_shared<webrtc::SimulcastEncoderAdapter>(
        webrtc::CreateEnvironment(), internal_encoder_factory_.get(), nullptr,
        format);
  } else {
    encoder.reset(create(format).release());
  }
  return std::make_unique<AlignedEncoderAdapter>(encoder, 16, 16);
}
