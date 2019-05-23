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

#ifndef IL_H264_ENCODER_H_
#define IL_H264_ENCODER_H_

extern "C"
{
#include "bcm_host.h"
#include "ilclient.h"
}

#include <memory>
#include <vector>
#include <list>

#include "api/video_codecs/video_encoder.h"
#include "rtc_base/critical_section.h"
#include "common_video/h264/h264_bitstream_parser.h"
#include "common_video/include/bitrate_adjuster.h"
#include "modules/video_coding/codecs/h264/include/h264.h"
#include "modules/video_coding/utility/quality_scaler.h"
#include "rtc_base/event.h"

class ProcessThread;

class ILH264Encoder : public webrtc::VideoEncoder
{
public:
  explicit ILH264Encoder(const cricket::VideoCodec &codec);
  ~ILH264Encoder() override;

  int32_t InitEncode(const webrtc::VideoCodec *codec_settings,
                     int32_t number_of_cores,
                     size_t max_payload_size) override;
  int32_t RegisterEncodeCompleteCallback(
      webrtc::EncodedImageCallback *callback) override;
  int32_t Release() override;
  int32_t Encode(const webrtc::VideoFrame &frame,
                 const std::vector<webrtc::VideoFrameType> *frame_types) override;
  void SetRates(const RateControlParameters& parameters) override;
  webrtc::VideoEncoder::EncoderInfo GetEncoderInfo() const override;

private:
  int32_t OMX_Configure();
  void OMX_Release();

  void SetBitrateBps(uint32_t bitrate_bps);

  int32_t DrainEncodedData();

  webrtc::EncodedImageCallback *callback_;
  ILCLIENT_T *ilclient_;
  COMPONENT_T *video_encode_;
  webrtc::BitrateAdjuster bitrate_adjuster_;
  webrtc::H264PacketizationMode packetization_mode_;
  uint32_t target_bitrate_bps_;
  uint32_t encoder_bitrate_bps_;
  int32_t width_;
  int32_t height_;
  int32_t configured_width_;
  int32_t configured_height_;

  webrtc::H264BitstreamParser h264_bitstream_parser_;

  COMPONENT_T *list_[5];

  webrtc::EncodedImage encoded_image_;
  std::unique_ptr<uint8_t[]> encoded_image_buffer_;

  bool omx_configured_;
  bool omx_reconfigure_;

  bool drop_next_frame_;
};

#endif // IL_H264_ENCODER_H_
