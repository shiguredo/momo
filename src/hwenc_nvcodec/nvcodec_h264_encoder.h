#ifndef NVCODEC_H264_ENCODER_H_
#define NVCODEC_H264_ENCODER_H_

#include <chrono>
#include <memory>
#include <queue>

#include "api/video_codecs/video_encoder.h"
#include "common_video/h264/h264_bitstream_parser.h"
#include "common_video/include/bitrate_adjuster.h"
#include "modules/video_coding/codecs/h264/include/h264.h"
#include "rtc_base/critical_section.h"

class ProcessThread;

class NvCodecH264Encoder : public webrtc::VideoEncoder {
 public:
  explicit NvCodecH264Encoder(const cricket::VideoCodec& codec);
  ~NvCodecH264Encoder() override;

  int32_t InitEncode(const webrtc::VideoCodec* codec_settings,
                     int32_t number_of_cores,
                     size_t max_payload_size) override;
  int32_t RegisterEncodeCompleteCallback(
      webrtc::EncodedImageCallback* callback) override;
  int32_t Release() override;
  int32_t Encode(
      const webrtc::VideoFrame& frame,
      const std::vector<webrtc::VideoFrameType>* frame_types) override;
  void SetRates(const RateControlParameters& parameters) override;
  webrtc::VideoEncoder::EncoderInfo GetEncoderInfo() const override;
};

#endif  // NVCODEC_H264_ENCODER_H_
