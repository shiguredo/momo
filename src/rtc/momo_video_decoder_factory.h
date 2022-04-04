#ifndef MOMO_VIDEO_DECODER_FACTORY_H_
#define MOMO_VIDEO_DECODER_FACTORY_H_

#include <memory>
#include <vector>

// WebRTC
#include <api/video_codecs/video_decoder_factory.h>

#include "video_codec_info.h"

#if USE_NVCODEC_ENCODER
#include "cuda/cuda_context.h"
#endif

class MomoVideoDecoderFactory : public webrtc::VideoDecoderFactory {
  VideoCodecInfo::Type vp8_decoder_;
  VideoCodecInfo::Type vp9_decoder_;
  VideoCodecInfo::Type av1_decoder_;
  VideoCodecInfo::Type h264_decoder_;
  std::unique_ptr<webrtc::VideoDecoderFactory> video_decoder_factory_;

 public:
  MomoVideoDecoderFactory(VideoCodecInfo::Type vp8_decoder,
                          VideoCodecInfo::Type vp9_decoder,
                          VideoCodecInfo::Type av1_decoder,
                          VideoCodecInfo::Type h264_decoder
#if USE_NVCODEC_ENCODER
                          ,
                          std::shared_ptr<CudaContext> cuda_context
#endif
  );
  virtual ~MomoVideoDecoderFactory() {}

  std::vector<webrtc::SdpVideoFormat> GetSupportedFormats() const override;

  std::unique_ptr<webrtc::VideoDecoder> CreateVideoDecoder(
      const webrtc::SdpVideoFormat& format) override;
#if USE_NVCODEC_ENCODER
  std::shared_ptr<CudaContext> cuda_context_;
#endif
};

#endif  // MOMO_VIDEO_DECODER_FACTORY_H_
