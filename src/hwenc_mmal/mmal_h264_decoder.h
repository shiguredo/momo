/*
 *  Copyright (c) 2015 The WebRTC project authors. All Rights Reserved.
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 *
 */

#ifndef MMAL_H264_DECODER_H_
#define MMAL_H264_DECODER_H_

extern "C" {
#include <bcm_host.h>
#include <interface/mmal/mmal.h>
#include <interface/mmal/mmal_format.h>
#include <interface/mmal/util/mmal_connection.h>
#include <interface/mmal/util/mmal_default_components.h>
#include <interface/mmal/util/mmal_util.h>
#include <interface/mmal/util/mmal_util_params.h>
#include <interface/vcos/vcos.h>
}

// WebRTC
#include <api/video_codecs/video_decoder.h>
#include <common_video/include/video_frame_buffer_pool.h>
#include <rtc_base/synchronization/mutex.h>

class MMALH264Decoder : public webrtc::VideoDecoder {
 public:
  MMALH264Decoder();
  ~MMALH264Decoder() override;

  int32_t InitDecode(const webrtc::VideoCodec* codec_settings,
                     int32_t number_of_cores) override;

  int32_t Decode(const webrtc::EncodedImage& input_image,
                 bool missing_frames,
                 int64_t render_time_ms) override;

  int32_t RegisterDecodeCompleteCallback(
      webrtc::DecodedImageCallback* callback) override;

  int32_t Release() override;

  const char* ImplementationName() const override;

 private:
  void FillOutputBuffer();
  static void MMALInputCallbackFunction(MMAL_PORT_T* port,
                                        MMAL_BUFFER_HEADER_T* buffer);
  static void MMALOutputCallbackFunction(MMAL_PORT_T* port,
                                         MMAL_BUFFER_HEADER_T* buffer);
  void MMALOutputCallback(MMAL_PORT_T* port, MMAL_BUFFER_HEADER_T* buffer);
  void SendFrame(MMAL_BUFFER_HEADER_T* buffer);
  int32_t MMALConfigure();
  void MMALRelease();

  webrtc::Mutex config_lock_;
  MMAL_COMPONENT_T* decoder_;
  MMAL_POOL_T* pool_in_;
  MMAL_POOL_T* pool_out_;
  int32_t width_;
  int32_t height_;
  webrtc::DecodedImageCallback* decode_complete_callback_;
  webrtc::VideoFrameBufferPool buffer_pool_;
};

#endif  // MMAL_H264_DECODER_H_
