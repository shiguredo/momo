#include "nvcodec_h264_encoder.h"

const int kLowH264QpThreshold = 34;
const int kHighH264QpThreshold = 40;

NvCodecH264Encoder::NvCodecH264Encoder(const cricket::VideoCodec& codec){}
NvCodecH264Encoder::~NvCodecH264Encoder() {}
int32_t NvCodecH264Encoder::InitEncode(const webrtc::VideoCodec* codec_settings,
                     int32_t number_of_cores,
                     size_t max_payload_size) { return 0; }
int32_t NvCodecH264Encoder::RegisterEncodeCompleteCallback(
      webrtc::EncodedImageCallback* callback) { return 0; }
int32_t NvCodecH264Encoder::Release() { return 0; }
int32_t NvCodecH264Encoder::Encode(
      const webrtc::VideoFrame& frame,
      const std::vector<webrtc::VideoFrameType>* frame_types) { return 0; }
void NvCodecH264Encoder::SetRates(const RateControlParameters& parameters) {}

webrtc::VideoEncoder::EncoderInfo NvCodecH264Encoder::GetEncoderInfo() const {
  EncoderInfo info;
  info.supports_native_handle = true;
  info.implementation_name = "Jetson H264";
  info.scaling_settings =
      VideoEncoder::ScalingSettings(kLowH264QpThreshold, kHighH264QpThreshold);
  info.is_hardware_accelerated = true;
  info.has_internal_source = false;
  return info;
}