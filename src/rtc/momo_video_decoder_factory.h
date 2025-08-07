#ifndef MOMO_VIDEO_DECODER_FACTORY_H_
#define MOMO_VIDEO_DECODER_FACTORY_H_

#include <memory>
#include <vector>

// WebRTC
#include <api/environment/environment.h>
#include <api/video_codecs/video_decoder_factory.h>

#include "video_codec_info.h"

#if defined(USE_NVCODEC_ENCODER)
#include "sora/cuda_context.h"
#endif
#if defined(USE_VPL_ENCODER)
#include "sora/vpl_session.h"
#endif
#if defined(USE_AMF_ENCODER)
#include "../hwenc_amf/amf_context.h"
#endif

struct MomoVideoDecoderFactoryConfig {
  VideoCodecInfo::Type vp8_decoder;
  VideoCodecInfo::Type vp9_decoder;
  VideoCodecInfo::Type av1_decoder;
  VideoCodecInfo::Type h264_decoder;
  VideoCodecInfo::Type h265_decoder;
#if defined(USE_NVCODEC_ENCODER)
  std::shared_ptr<sora::CudaContext> cuda_context;
#endif
#if defined(USE_AMF_ENCODER)
  std::shared_ptr<momo::AMFContext> amf_context;
#endif
};

class MomoVideoDecoderFactory : public webrtc::VideoDecoderFactory {
  MomoVideoDecoderFactoryConfig config_;
  std::unique_ptr<webrtc::VideoDecoderFactory> video_decoder_factory_;

 public:
  MomoVideoDecoderFactory(const MomoVideoDecoderFactoryConfig& config);
  virtual ~MomoVideoDecoderFactory() {}

  std::vector<webrtc::SdpVideoFormat> GetSupportedFormats() const override;

  std::unique_ptr<webrtc::VideoDecoder> Create(
      const webrtc::Environment& env,
      const webrtc::SdpVideoFormat& format) override;
};

#endif  // MOMO_VIDEO_DECODER_FACTORY_H_
