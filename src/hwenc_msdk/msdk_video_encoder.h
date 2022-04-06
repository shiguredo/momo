#ifndef MSDK_VIDEO_ENCODER_H_
#define MSDK_VIDEO_ENCODER_H_

#include <mutex>
#include <vector>

// WebRTC
#include <api/video_codecs/video_encoder.h>
#include <common_video/h264/h264_bitstream_parser.h>
#include <common_video/include/bitrate_adjuster.h>
#include <modules/video_coding/codecs/h264/include/h264.h>
#include <rtc_base/logging.h>
#include <rtc_base/synchronization/mutex.h>

// msdk
#include <mfx/mfxvideo++.h>
#include <mfx/mfxvp8.h>

#include "vaapi_utils_drm.h"

class MsdkVideoEncoder : public webrtc::VideoEncoder {
 public:
  explicit MsdkVideoEncoder(const cricket::VideoCodec& codec);
  ~MsdkVideoEncoder() override;

  // MFX_CODEC_VP8
  // MFX_CODEC_VP9
  // MFX_CODEC_AVC
  // MFX_CODEC_AV1
  static bool IsSupported(mfxU32 codec);

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

 private:
  std::mutex mutex_;
  webrtc::EncodedImageCallback* callback_ = nullptr;
  webrtc::BitrateAdjuster bitrate_adjuster_;
  uint32_t target_bitrate_bps_ = 0;
  uint32_t max_bitrate_bps_ = 0;
  bool reconfigure_needed_ = false;
  bool use_native_ = false;
  uint32_t width_ = 0;
  uint32_t height_ = 0;
  uint32_t framerate_ = 0;
  webrtc::VideoCodecMode mode_ = webrtc::VideoCodecMode::kRealtimeVideo;
  std::vector<std::vector<uint8_t>> v_packet_;
  webrtc::EncodedImage encoded_image_;
  webrtc::H264BitstreamParser h264_bitstream_parser_;

  int32_t InitMediaSDK();
  int32_t ReleaseMediaSDK();

  std::vector<uint8_t> surface_buffer_;
  std::vector<mfxFrameSurface1> surfaces_;

  std::unique_ptr<DRMLibVA> libva_;
  MFXVideoSession session_;
  mfxFrameAllocRequest alloc_request_;
  std::unique_ptr<MFXVideoENCODE> encoder_;
  std::vector<uint8_t> bitstream_buffer_;
  mfxBitstream bitstream_;
  mfxFrameInfo frame_info_;
};

#endif  // MSDK_VIDEO_ENCODER_H_