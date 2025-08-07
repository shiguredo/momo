#ifndef MOMO_VIDEO_ENCODER_FACTORY_H_
#define MOMO_VIDEO_ENCODER_FACTORY_H_

#include <memory>
#include <vector>

// WebRTC
#include <api/environment/environment.h>
#include <api/video_codecs/sdp_video_format.h>
#include <api/video_codecs/video_encoder.h>
#include <api/video_codecs/video_encoder_factory.h>

#include "video_codec_info.h"

#if defined(USE_NVCODEC_ENCODER)
#include "sora/cuda_context.h"
#endif

#if defined(USE_VPL_ENCODER)
#include "sora/vpl_session.h"
#endif

#if defined(USE_AMF_ENCODER)
#include <sora/amf_context.h>
#endif

struct MomoVideoEncoderFactoryConfig {
  VideoCodecInfo::Type vp8_encoder;
  VideoCodecInfo::Type vp9_encoder;
  VideoCodecInfo::Type av1_encoder;
  VideoCodecInfo::Type h264_encoder;
  VideoCodecInfo::Type h265_encoder;
  bool simulcast;
  bool hardware_encoder_only;
#if defined(USE_NVCODEC_ENCODER)
  std::shared_ptr<sora::CudaContext> cuda_context;
#endif
#if defined(USE_AMF_ENCODER)
  std::shared_ptr<sora::AMFContext> amf_context;
#endif
  std::string openh264;
};

class MomoVideoEncoderFactory : public webrtc::VideoEncoderFactory {
  MomoVideoEncoderFactoryConfig config_;
  std::unique_ptr<webrtc::VideoEncoderFactory> video_encoder_factory_;
  std::unique_ptr<MomoVideoEncoderFactory> internal_encoder_factory_;

 public:
  MomoVideoEncoderFactory(const MomoVideoEncoderFactoryConfig& config);
  virtual ~MomoVideoEncoderFactory() {}

  std::vector<webrtc::SdpVideoFormat> GetSupportedFormats() const override;

  std::unique_ptr<webrtc::VideoEncoder> Create(
      const webrtc::Environment& env,
      const webrtc::SdpVideoFormat& format) override;

 private:
  std::unique_ptr<webrtc::VideoEncoder> CreateInternal(
      const webrtc::Environment& env,
      const webrtc::SdpVideoFormat& format);
  std::unique_ptr<webrtc::VideoEncoder> WithSimulcast(
      const webrtc::SdpVideoFormat& format,
      std::function<std::unique_ptr<webrtc::VideoEncoder>(
          const webrtc::SdpVideoFormat&)> create);
};

#endif  // MOMO_VIDEO_ENCODER_FACTORY_H_
