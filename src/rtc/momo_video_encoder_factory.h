#ifndef MOMO_VIDEO_ENCODER_FACTORY_H_
#define MOMO_VIDEO_ENCODER_FACTORY_H_

#include <memory>
#include <vector>

// WebRTC
#include <api/video_codecs/sdp_video_format.h>
#include <api/video_codecs/video_encoder.h>
#include <api/video_codecs/video_encoder_factory.h>

#include "video_codec_info.h"

class MomoVideoEncoderFactory : public webrtc::VideoEncoderFactory {
  VideoCodecInfo::Type vp8_encoder_;
  VideoCodecInfo::Type vp9_encoder_;
  VideoCodecInfo::Type av1_encoder_;
  VideoCodecInfo::Type h264_encoder_;
  VideoCodecInfo::Type h265_encoder_;
  std::unique_ptr<webrtc::VideoEncoderFactory> video_encoder_factory_;
  std::unique_ptr<MomoVideoEncoderFactory> internal_encoder_factory_;

 public:
  MomoVideoEncoderFactory(VideoCodecInfo::Type vp8_encoder,
                          VideoCodecInfo::Type vp9_encoder,
                          VideoCodecInfo::Type av1_encoder,
                          VideoCodecInfo::Type h264_encoder,
                          VideoCodecInfo::Type h265_encoder,
                          bool simulcast);
  virtual ~MomoVideoEncoderFactory() {}

  std::vector<webrtc::SdpVideoFormat> GetSupportedFormats() const override;

  CodecInfo QueryVideoEncoder(
      const webrtc::SdpVideoFormat& format) const override;

  std::unique_ptr<webrtc::VideoEncoder> CreateVideoEncoder(
      const webrtc::SdpVideoFormat& format) override;

 private:
  std::unique_ptr<webrtc::VideoEncoder> WithSimulcast(
      const webrtc::SdpVideoFormat& format,
      std::function<std::unique_ptr<webrtc::VideoEncoder>(
          const webrtc::SdpVideoFormat&)> create);
};

#endif  // MOMO_VIDEO_ENCODER_FACTORY_H_
