#ifndef NVCODEC_H264_ENCODER_H_
#define NVCODEC_H264_ENCODER_H_

#ifdef _WIN32
#include <d3d11.h>
#include <wrl.h>
#endif

#include <chrono>
#include <memory>
#include <mutex>
#include <queue>

// WebRTC
#include <api/video_codecs/video_encoder.h>
#include <common_video/h264/h264_bitstream_parser.h>
#include <common_video/include/bitrate_adjuster.h>
#include <modules/video_coding/codecs/h264/include/h264.h>

// NvCodec
#ifdef _WIN32
#include <NvEncoder/NvEncoderD3D11.h>
#endif
#ifdef __linux__
#include "nvcodec_h264_encoder_cuda.h"
#endif

class NvCodecH264Encoder : public webrtc::VideoEncoder {
 public:
  explicit NvCodecH264Encoder(const cricket::VideoCodec& codec);
  ~NvCodecH264Encoder() override;

  static bool IsSupported();

  int32_t InitEncode(const webrtc::VideoCodec* codec_settings,
                     int32_t number_of_cores,
                     size_t max_payload_size) override;
  int32_t RegisterEncodeCompleteCallback(
      webrtc::EncodedImageCallback* callback) override;
  int32_t Release() override;
  int32_t Encode(
      const webrtc::VideoFrame& frame,
      const std::vector<webrtc::VideoFrameType>* frame_types) override;
  void SetRates(
      const webrtc::VideoEncoder::RateControlParameters& parameters) override;
  webrtc::VideoEncoder::EncoderInfo GetEncoderInfo() const override;

 private:
  std::mutex mutex_;
  webrtc::EncodedImageCallback* callback_ = nullptr;
  webrtc::BitrateAdjuster bitrate_adjuster_;
  uint32_t target_bitrate_bps_ = 0;
  uint32_t max_bitrate_bps_ = 0;

  int32_t InitNvEnc();
  int32_t ReleaseNvEnc();
  webrtc::H264BitstreamParser h264_bitstream_parser_;

#ifdef _WIN32
  Microsoft::WRL::ComPtr<ID3D11Device> id3d11_device_;
  Microsoft::WRL::ComPtr<ID3D11DeviceContext> id3d11_context_;
  Microsoft::WRL::ComPtr<ID3D11Texture2D> id3d11_texture_;
  std::unique_ptr<NvEncoderD3D11> nv_encoder_;
#endif
#ifdef __linux__
  std::unique_ptr<NvCodecH264EncoderCuda> cuda_;
  std::unique_ptr<NvEncoder> nv_encoder_;
#endif
  bool reconfigure_needed_ = false;
  bool use_native_ = false;
  uint32_t width_ = 0;
  uint32_t height_ = 0;
  uint32_t framerate_ = 0;
  webrtc::VideoCodecMode mode_ = webrtc::VideoCodecMode::kRealtimeVideo;
  NV_ENC_INITIALIZE_PARAMS initialize_params_;
  std::vector<std::vector<uint8_t>> v_packet_;
  webrtc::EncodedImage encoded_image_;
};

#endif  // NVCODEC_H264_ENCODER_H_
