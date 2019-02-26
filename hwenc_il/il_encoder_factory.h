#ifndef IL_ENCODER_FACTORY_H_
#define IL_ENCODER_FACTORY_H_

#include <memory>
#include <vector>

#include "api/video_codecs/sdp_video_format.h"
#include "api/video_codecs/video_encoder.h"
#include "api/video_codecs/video_encoder_factory.h"

class ILVideoEncoderFactory : public webrtc::VideoEncoderFactory
{
  public:
    ILVideoEncoderFactory() {}
    virtual ~ILVideoEncoderFactory() {}

    std::vector<webrtc::SdpVideoFormat> GetSupportedFormats() const override;

    CodecInfo QueryVideoEncoder(const webrtc::SdpVideoFormat& format) const override;

    std::unique_ptr<webrtc::VideoEncoder> CreateVideoEncoder(
      const webrtc::SdpVideoFormat& format) override;
};

#endif // IL_ENCODER_FACTORY_H_