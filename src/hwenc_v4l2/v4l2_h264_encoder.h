#ifndef V4L2_H264_ENCODER_H_
#define V4L2_H264_ENCODER_H_

#include <chrono>
#include <memory>
#include <mutex>
#include <queue>

#include <linux/videodev2.h>

// WebRTC
#include <api/video_codecs/video_encoder.h>
#include <common_video/h264/h264_bitstream_parser.h>
#include <common_video/include/bitrate_adjuster.h>
#include <common_video/libyuv/include/webrtc_libyuv.h>
#include <modules/video_coding/codecs/h264/include/h264.h>
#include <rtc_base/platform_thread.h>
#include <rtc_base/synchronization/mutex.h>

#include "v4l2_converter.h"

class V4L2H264Encoder : public webrtc::VideoEncoder {
 public:
  explicit V4L2H264Encoder(const cricket::VideoCodec& codec);
  ~V4L2H264Encoder() override;

  int32_t InitEncode(const webrtc::VideoCodec* codec_settings,
                     const webrtc::VideoEncoder::Settings& settings) override;
  int32_t RegisterEncodeCompleteCallback(
      webrtc::EncodedImageCallback* callback) override;
  int32_t Release() override;
  void SetRates(const RateControlParameters& parameters) override;
  webrtc::VideoEncoder::EncoderInfo GetEncoderInfo() const override;
  int32_t Encode(
      const webrtc::VideoFrame& frame,
      const std::vector<webrtc::VideoFrameType>* frame_types) override;

 private:
  int32_t Configure(webrtc::VideoFrameBuffer::Type type,
                    webrtc::VideoType video_type,
                    int32_t width,
                    int32_t height,
                    int32_t stride,
                    int32_t scaled_width,
                    int32_t scaled_height);
  void SetBitrateBps(uint32_t bitrate_bps);
  void SetFramerateFps(double framerate_fps);
  int32_t SendFrame(const webrtc::VideoFrame& frame,
                    unsigned char* buffer,
                    size_t size,
                    int64_t timestamp_us,
                    bool is_key_frame);

 private:
  std::shared_ptr<V4L2DecodeConverter> jpeg_decoder_;
  std::shared_ptr<V4L2ScaleConverter> scaler_;
  std::shared_ptr<V4L2H264EncodeConverter> h264_encoder_;

  webrtc::VideoFrameBuffer::Type configured_type_;
  int32_t configured_width_;
  int32_t configured_height_;

  webrtc::EncodedImageCallback* callback_;
  std::mutex callback_mutex_;
  webrtc::BitrateAdjuster bitrate_adjuster_;
  uint32_t target_bitrate_bps_;
  uint32_t configured_bitrate_bps_;
  double target_framerate_fps_;
  int32_t configured_framerate_fps_;

  webrtc::H264BitstreamParser h264_bitstream_parser_;

  webrtc::EncodedImage encoded_image_;
};

#endif  // V4L2_H264_ENCODER_H_
