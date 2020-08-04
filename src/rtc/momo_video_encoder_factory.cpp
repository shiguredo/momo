#include "momo_video_encoder_factory.h"

// WebRTC
#include <absl/memory/memory.h>
#include <absl/strings/match.h>
#include <api/video_codecs/sdp_video_format.h>
#include <media/base/codec.h>
#include <media/base/media_constants.h>
#include <media/base/vp9_profile.h>
#include <media/engine/simulcast_encoder_adapter.h>
#include <modules/video_coding/codecs/h264/include/h264.h>
#include <modules/video_coding/codecs/vp8/include/vp8.h>
#include <modules/video_coding/codecs/vp9/include/vp9.h>
#include <rtc_base/logging.h>

#if !defined(__arm__) || defined(__aarch64__) || defined(__ARM_NEON__)
#include <modules/video_coding/codecs/av1/libaom_av1_encoder.h>
#endif

#if defined(__APPLE__)
#include "mac_helper/objc_codec_factory_helper.h"
#endif

#if USE_MMAL_ENCODER
#include "hwenc_mmal/mmal_h264_encoder.h"
#endif
#if USE_JETSON_ENCODER
#include "hwenc_jetson/jetson_video_encoder.h"
#endif
#if USE_NVCODEC_ENCODER
#include "hwenc_nvcodec/nvcodec_h264_encoder.h"
#endif

#include "h264_format.h"

MomoVideoEncoderFactory::MomoVideoEncoderFactory(
    VideoCodecInfo::Type vp8_encoder,
    VideoCodecInfo::Type vp9_encoder,
    VideoCodecInfo::Type av1_encoder,
    VideoCodecInfo::Type h264_encoder,
    VideoCodecInfo::Type h265_encoder,
    bool simulcast)
    : vp8_encoder_(vp8_encoder),
      vp9_encoder_(vp9_encoder),
      av1_encoder_(av1_encoder),
      h264_encoder_(h264_encoder),
      h265_encoder_(h265_encoder) {
#if defined(__APPLE__)
  video_encoder_factory_ = CreateObjCEncoderFactory();
#endif
  if (simulcast) {
    internal_encoder_factory_.reset(new MomoVideoEncoderFactory(
        vp8_encoder, vp9_encoder, av1_encoder, h264_encoder, h265_encoder, false));
  }
}
std::vector<webrtc::SdpVideoFormat>
MomoVideoEncoderFactory::GetSupportedFormats() const {
  std::vector<webrtc::SdpVideoFormat> supported_codecs;

  // VP8
  // 今のところ Software のみ
  if (vp8_encoder_ == VideoCodecInfo::Type::Software) {
    supported_codecs.push_back(webrtc::SdpVideoFormat(cricket::kVp8CodecName));
  }

  // VP9
  if (vp9_encoder_ == VideoCodecInfo::Type::Software) {
    for (const webrtc::SdpVideoFormat& format : webrtc::SupportedVP9Codecs()) {
      supported_codecs.push_back(format);
    }
  } else if (vp9_encoder_ == VideoCodecInfo::Type::Jetson) {
#if USE_JETSON_ENCODER
    supported_codecs.push_back(webrtc::SdpVideoFormat(
        cricket::kVp9CodecName,
        {{webrtc::kVP9FmtpProfileId,
          webrtc::VP9ProfileToString(webrtc::VP9Profile::kProfile0)}}));
#endif
  }

  // AV1
  // 今のところ Software のみ
  if (av1_encoder_ == VideoCodecInfo::Type::Software) {
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

  if (h264_encoder_ == VideoCodecInfo::Type::VideoToolbox) {
    // VideoToolbox の場合は video_encoder_factory_ から H264 を拾ってくる
    for (auto format : video_encoder_factory_->GetSupportedFormats()) {
      if (absl::EqualsIgnoreCase(format.name, cricket::kH264CodecName)) {
        supported_codecs.push_back(format);
      }
    }
  } else if (h264_encoder_ == VideoCodecInfo::Type::NVIDIA) {
#if USE_NVCODEC_ENCODER
    // NVIDIA の場合は対応してる場合のみ追加
    if (NvCodecH264Encoder::IsSupported()) {
      for (const webrtc::SdpVideoFormat& format : h264_codecs) {
        supported_codecs.push_back(format);
      }
    }
#endif
  } else if (h264_encoder_ != VideoCodecInfo::Type::NotSupported) {
    // その他のエンコーダの場合は手動で追加
    for (const webrtc::SdpVideoFormat& format : h264_codecs) {
      supported_codecs.push_back(format);
    }
  }

  // H265
  if (h265_encoder_ == VideoCodecInfo::Type::VideoToolbox) {
    // VideoToolbox の場合は video_encoder_factory_ から H265 を拾ってくる
    for (auto format : video_encoder_factory_->GetSupportedFormats()) {
      if (absl::EqualsIgnoreCase(format.name, cricket::kH265CodecName)) {
        supported_codecs.push_back(format);
      }
    }
  } else if (h264_encoder_ != VideoCodecInfo::Type::NotSupported) {
    // その他のエンコーダの場合は手動で追加
    supported_codecs.push_back(webrtc::SdpVideoFormat(cricket::kH265CodecName));
  }
  return supported_codecs;
}

webrtc::VideoEncoderFactory::CodecInfo
MomoVideoEncoderFactory::QueryVideoEncoder(
    const webrtc::SdpVideoFormat& format) const {
  CodecInfo info;
  info.has_internal_source = false;
  if (absl::EqualsIgnoreCase(format.name, cricket::kVp8CodecName)) {
    assert(vp8_encoder_ != VideoCodecInfo::Type::NotSupported);
    info.is_hardware_accelerated =
        vp8_encoder_ != VideoCodecInfo::Type::Software;
  } else if (absl::EqualsIgnoreCase(format.name, cricket::kVp9CodecName)) {
    assert(vp9_encoder_ != VideoCodecInfo::Type::NotSupported);
    info.is_hardware_accelerated =
        vp9_encoder_ != VideoCodecInfo::Type::Software;
  } else if (absl::EqualsIgnoreCase(format.name, cricket::kAv1CodecName)) {
    assert(av1_encoder_ != VideoCodecInfo::Type::NotSupported);
    info.is_hardware_accelerated =
        av1_encoder_ != VideoCodecInfo::Type::Software;
  } else if (absl::EqualsIgnoreCase(format.name, cricket::kH264CodecName)) {
    assert(h264_encoder_ != VideoCodecInfo::Type::NotSupported);
    if (h264_encoder_ == VideoCodecInfo::Type::VideoToolbox) {
      return video_encoder_factory_->QueryVideoEncoder(format);
    } else {
      info.is_hardware_accelerated =
          h264_encoder_ != VideoCodecInfo::Type::Software;
    }
  } else if (absl::EqualsIgnoreCase(format.name, cricket::kH265CodecName)) {
    assert(h265_encoder_ != VideoCodecInfo::Type::NotSupported);
    if (h265_encoder_ == VideoCodecInfo::Type::VideoToolbox) {
      return video_encoder_factory_->QueryVideoEncoder(format);
    } else {
      info.is_hardware_accelerated =
          h265_encoder_ != VideoCodecInfo::Type::Software;
    }
  } else {
    RTC_LOG(LS_ERROR) << "Unknown format: " << format.name;
    std::exit(1);
  }
  return info;
}

std::unique_ptr<webrtc::VideoEncoder>
MomoVideoEncoderFactory::CreateVideoEncoder(
    const webrtc::SdpVideoFormat& format) {
  if (absl::EqualsIgnoreCase(format.name, cricket::kVp8CodecName)) {
    if (vp8_encoder_ == VideoCodecInfo::Type::Software) {
      return webrtc::VP8Encoder::Create();
    }
  }

  if (absl::EqualsIgnoreCase(format.name, cricket::kVp9CodecName)) {
    if (vp9_encoder_ == VideoCodecInfo::Type::Software) {
      return webrtc::VP9Encoder::Create(cricket::VideoCodec(format));
    }
#if USE_JETSON_ENCODER
    if (vp9_encoder_ == VideoCodecInfo::Type::Jetson) {
      return WithSimulcast(format, [](const webrtc::SdpVideoFormat& format) {
        return std::unique_ptr<webrtc::VideoEncoder>(
            absl::make_unique<JetsonVideoEncoder>(cricket::VideoCodec(format)));
      });
    }
#endif
  }

  if (absl::EqualsIgnoreCase(format.name, cricket::kAv1CodecName)) {
#if !defined(__arm__) || defined(__aarch64__) || defined(__ARM_NEON__)
    if (av1_encoder_ == VideoCodecInfo::Type::Software) {
      return webrtc::CreateLibaomAv1Encoder();
    }
#endif
  }

  if (absl::EqualsIgnoreCase(format.name, cricket::kH264CodecName)) {
#if defined(__APPLE__)
    if (h264_encoder_ == VideoCodecInfo::Type::VideoToolbox) {
      return WithSimulcast(
          format, [this](const webrtc::SdpVideoFormat& format) {
            return video_encoder_factory_->CreateVideoEncoder(format);
          });
    }
#endif

#if USE_MMAL_ENCODER
    if (h264_encoder_ == VideoCodecInfo::Type::MMAL) {
      return WithSimulcast(format, [](const webrtc::SdpVideoFormat& format) {
        return std::unique_ptr<webrtc::VideoEncoder>(
            absl::make_unique<MMALH264Encoder>(cricket::VideoCodec(format)));
      });
    }
#endif

#if USE_JETSON_ENCODER
    if (h264_encoder_ == VideoCodecInfo::Type::Jetson) {
      return WithSimulcast(format, [](const webrtc::SdpVideoFormat& format) {
        return std::unique_ptr<webrtc::VideoEncoder>(
            absl::make_unique<JetsonVideoEncoder>(cricket::VideoCodec(format)));
      });
    }
#endif

#if USE_NVCODEC_ENCODER
    if (h264_encoder_ == VideoCodecInfo::Type::NVIDIA &&
        NvCodecH264Encoder::IsSupported()) {
      return WithSimulcast(format, [](const webrtc::SdpVideoFormat& format) {
        return std::unique_ptr<webrtc::VideoEncoder>(
            absl::make_unique<NvCodecH264Encoder>(cricket::VideoCodec(format)));
      });
    }
#endif
  }

  if (absl::EqualsIgnoreCase(format.name, cricket::kH265CodecName)) {
#if defined(__APPLE__)
    if (h265_encoder_ == VideoCodecInfo::Type::VideoToolbox) {
      return video_encoder_factory_->CreateVideoEncoder(format);
    }
#endif

#if USE_JETSON_ENCODER
    if (h265_encoder_ == VideoCodecInfo::Type::Jetson) {
      return std::unique_ptr<webrtc::VideoEncoder>(
          absl::make_unique<JetsonVideoEncoder>(cricket::VideoCodec(format)));
    }
#endif
  }

  RTC_LOG(LS_ERROR) << "Trying to created encoder of unsupported format "
                    << format.name;
  return nullptr;
}

std::unique_ptr<webrtc::VideoEncoder> MomoVideoEncoderFactory::WithSimulcast(
    const webrtc::SdpVideoFormat& format,
    std::function<std::unique_ptr<webrtc::VideoEncoder>(
        const webrtc::SdpVideoFormat&)> create) {
  if (internal_encoder_factory_) {
    return std::unique_ptr<webrtc::VideoEncoder>(
        new webrtc::SimulcastEncoderAdapter(internal_encoder_factory_.get(),
                                            format));
  } else {
    return create(format);
  }
}
