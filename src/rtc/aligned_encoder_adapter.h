#ifndef ALIGNED_ENCODER_ADAPTER_H_
#define ALIGNED_ENCODER_ADAPTER_H_

#include <absl/types/optional.h>
#include <api/fec_controller_override.h>
#include <api/sequence_checker.h>
#include <api/video_codecs/sdp_video_format.h>
#include <api/video_codecs/video_encoder.h>
#include <api/video_codecs/video_encoder_factory.h>
#include <common_video/framerate_controller.h>
#include <modules/video_coding/include/video_codec_interface.h>
#include <rtc_base/experiments/encoder_info_settings.h>
#include <rtc_base/system/no_unique_address.h>
#include <rtc_base/system/rtc_export.h>

class AlignedEncoderAdapter : public webrtc::VideoEncoder {
 public:
  AlignedEncoderAdapter(std::shared_ptr<webrtc::VideoEncoder> encoder,
                        int horizontal_alignment,
                        int vertical_alignment);

  void SetFecControllerOverride(
      webrtc::FecControllerOverride* fec_controller_override) override;
  int Release() override;
  int InitEncode(const webrtc::VideoCodec* codec_settings,
                 const webrtc::VideoEncoder::Settings& settings) override;
  int Encode(const webrtc::VideoFrame& input_image,
             const std::vector<webrtc::VideoFrameType>* frame_types) override;
  int RegisterEncodeCompleteCallback(
      webrtc::EncodedImageCallback* callback) override;
  void SetRates(const RateControlParameters& parameters) override;
  void OnPacketLossRateUpdate(float packet_loss_rate) override;
  void OnRttUpdate(int64_t rtt_ms) override;
  void OnLossNotification(const LossNotification& loss_notification) override;

  EncoderInfo GetEncoderInfo() const override;

 private:
  std::shared_ptr<webrtc::VideoEncoder> encoder_;
  int horizontal_alignment_;
  int vertical_alignment_;
  int width_;
  int height_;
};

#endif