#ifndef SORA_HWENC_V4L2_V4L2_H264_DECODER_H_
#define SORA_HWENC_V4L2_V4L2_H264_DECODER_H_

// WebRTC
#include <api/video_codecs/video_decoder.h>
#include <common_video/include/video_frame_buffer_pool.h>
#include <rtc_base/platform_thread.h>

#include "v4l2_converter.h"

namespace sora {

class V4L2H264Decoder : public webrtc::VideoDecoder {
 public:
  explicit V4L2H264Decoder(webrtc::VideoCodecType type);
  ~V4L2H264Decoder() override;

  static std::unique_ptr<V4L2H264Decoder> Create(webrtc::VideoCodecType type);
  static bool IsSupported(webrtc::VideoCodecType type);

  bool Configure(const Settings& settings) override;
  int32_t Decode(const webrtc::EncodedImage& input_image,
                 bool missing_frames,
                 int64_t render_time_ms) override;
  int32_t RegisterDecodeCompleteCallback(
      webrtc::DecodedImageCallback* callback) override;
  int32_t Release() override;
  const char* ImplementationName() const override;

 private:
  webrtc::DecodedImageCallback* decode_complete_callback_;
  std::shared_ptr<V4L2DecodeConverter> decoder_;
};

}  // namespace sora

#endif
