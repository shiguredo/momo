/*
 *  Copyright (c) 2015 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 *
 */

#ifndef MMAL_H264_ENCODER_H_
#define MMAL_H264_ENCODER_H_

extern "C"
{
#include "bcm_host.h"
#include "interface/mmal/mmal.h"
#include "interface/mmal/mmal_format.h"
#include "interface/mmal/util/mmal_connection.h"
#include "interface/mmal/util/mmal_default_components.h"
#include "interface/mmal/util/mmal_util_params.h"
#include "interface/mmal/util/mmal_util.h"
#include "interface/vcos/vcos.h"
}

#include <chrono>
#include <memory>
#include <queue>

#include "api/video_codecs/video_encoder.h"
#include "rtc_base/critical_section.h"
#include "common_video/h264/h264_bitstream_parser.h"
#include "common_video/include/bitrate_adjuster.h"
#include "modules/video_coding/codecs/h264/include/h264.h"

class ProcessThread;

class MMALH264Encoder : public webrtc::VideoEncoder
{
public:
  explicit MMALH264Encoder(const cricket::VideoCodec &codec);
  ~MMALH264Encoder() override;

  int32_t InitEncode(const webrtc::VideoCodec *codec_settings,
                     int32_t number_of_cores,
                     size_t max_payload_size) override;
  int32_t RegisterEncodeCompleteCallback(
      webrtc::EncodedImageCallback *callback) override;
  int32_t Release() override;
  int32_t Encode(const webrtc::VideoFrame &frame,
                 const std::vector<webrtc::VideoFrameType> *frame_types) override;
  void SetRates(const RateControlParameters &parameters) override;
  webrtc::VideoEncoder::EncoderInfo GetEncoderInfo() const override;

private:
  struct FrameParams {
    FrameParams(int32_t w,
                        int32_t h,
                        int64_t rtms,
                        int64_t ntpms,
                        int64_t ts,
                        webrtc::VideoRotation r,
                        absl::optional<webrtc::ColorSpace> c)
        : width(w), height(h), render_time_ms(rtms), ntp_time_ms(ntpms), timestamp(ts), rotation(r), color_space(c) {}

    int32_t width;
    int32_t height;
    int64_t render_time_ms;
    int64_t ntp_time_ms;
    int64_t timestamp;
    webrtc::VideoRotation rotation;
    absl::optional<webrtc::ColorSpace> color_space;
  };

  int32_t MMALConfigure();
  void MMALRelease();
  static void MMALInputCallbackFunction(MMAL_PORT_T* port, MMAL_BUFFER_HEADER_T* buffer);
  void MMALInputCallback(MMAL_PORT_T* port, MMAL_BUFFER_HEADER_T* buffer);
  static void MMALOutputCallbackFunction(MMAL_PORT_T* port, MMAL_BUFFER_HEADER_T* buffer);
  void MMALOutputCallback(MMAL_PORT_T* port, MMAL_BUFFER_HEADER_T* buffer);
  void SetBitrateBps(uint32_t bitrate_bps);
  int32_t SendFrame(unsigned char *buffer, size_t size);

  webrtc::EncodedImageCallback *callback_;
  MMAL_COMPONENT_T* decoder_;
  MMAL_COMPONENT_T* resizer_;
  MMAL_COMPONENT_T* encoder_;
  MMAL_CONNECTION_T *conn1_;
  MMAL_CONNECTION_T *conn2_;
  MMAL_QUEUE_T *queue_;
  MMAL_POOL_T *pool_in_;
  MMAL_POOL_T *pool_out_;
  webrtc::BitrateAdjuster bitrate_adjuster_;
  uint32_t target_bitrate_bps_;
  uint32_t configured_bitrate_bps_;
  int32_t raw_width_;
  int32_t raw_height_;
  int32_t width_;
  int32_t height_;
  int32_t configured_width_;
  int32_t configured_height_;
  int32_t stride_width_;
  int32_t stride_height_;
  bool use_native_;
  bool use_decoder_;

  webrtc::H264BitstreamParser h264_bitstream_parser_;


  rtc::CriticalSection frame_params_lock_;
  std::queue<std::unique_ptr<FrameParams>> frame_params_;
  webrtc::EncodedImage encoded_image_;
  std::unique_ptr<uint8_t[]> encoded_image_buffer_;
  size_t encoded_buffer_length_;
};

#endif // MMAL_H264_ENCODER_H_
