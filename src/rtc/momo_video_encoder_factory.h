#ifndef MOMO_VIDEO_ENCODER_FACTORY_H_
#define MOMO_VIDEO_ENCODER_FACTORY_H_

#include <memory>
#include <vector>

// WebRTC
#include <api/video_codecs/sdp_video_format.h>
#include <api/video_codecs/video_encoder.h>
#include <api/video_codecs/video_encoder_factory.h>

#include "video_codec_info.h"

#if defined(__linux__) && USE_NVCODEC_ENCODER
#include "cuda/cuda_context.h"
#endif

#if USE_MSDK_ENCODER
#include "hwenc_msdk/msdk_session.h"
#endif

struct MomoVideoEncoderFactoryConfig {
  VideoCodecInfo::Type vp8_encoder;
  VideoCodecInfo::Type vp9_encoder;
  VideoCodecInfo::Type av1_encoder;
  VideoCodecInfo::Type h264_encoder;
  bool simulcast;
  bool hardware_encoder_only;
#if defined(__linux__) && USE_NVCODEC_ENCODER
  std::shared_ptr<CudaContext> cuda_context;
#endif
};

class MomoVideoEncoderFactory : public webrtc::VideoEncoderFactory {
  MomoVideoEncoderFactoryConfig config_;
  std::unique_ptr<webrtc::VideoEncoderFactory> video_encoder_factory_;
  std::unique_ptr<MomoVideoEncoderFactory> internal_encoder_factory_;

 public:
  MomoVideoEncoderFactory(const MomoVideoEncoderFactoryConfig& config);
  virtual ~MomoVideoEncoderFactory() {}

  std::vector<webrtc::SdpVideoFormat> GetSupportedFormats() const override;

  std::unique_ptr<webrtc::VideoEncoder> CreateVideoEncoder(
      const webrtc::SdpVideoFormat& format) override;

 private:
  std::unique_ptr<webrtc::VideoEncoder> WithSimulcast(
      const webrtc::SdpVideoFormat& format,
      std::function<std::unique_ptr<webrtc::VideoEncoder>(
          const webrtc::SdpVideoFormat&)> create);
};

#endif  // MOMO_VIDEO_ENCODER_FACTORY_H_
