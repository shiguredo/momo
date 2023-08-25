#include "v4l2_h264_decoder.h"

#include <unistd.h>

// WebRTC
#include <modules/video_coding/include/video_error_codes.h>
#include <rtc_base/checks.h>
#include <rtc_base/logging.h>
#include <rtc_base/time_utils.h>
#include <system_wrappers/include/metrics.h>
#include <third_party/libyuv/include/libyuv/convert.h>

V4L2H264Decoder::V4L2H264Decoder(webrtc::VideoCodecType codec)
    : decoder_(nullptr), decode_complete_callback_(nullptr) {}

V4L2H264Decoder::~V4L2H264Decoder() {
  Release();
}

bool V4L2H264Decoder::Configure(const Settings& settings) {
  decoder_ = V4L2DecodeConverter::Create(V4L2_PIX_FMT_H264, false);
  if (decoder_ == nullptr) {
    RTC_LOG(LS_ERROR) << "Failed to create decoder";
    return false;
  }
  return true;
}

int32_t V4L2H264Decoder::Decode(const webrtc::EncodedImage& input_image,
                                bool missing_frames,
                                int64_t render_time_ms) {
  if (decoder_ == nullptr) {
    return WEBRTC_VIDEO_CODEC_UNINITIALIZED;
  }
  if (decode_complete_callback_ == NULL) {
    return WEBRTC_VIDEO_CODEC_UNINITIALIZED;
  }
  if (input_image.data() == NULL && input_image.size() > 0) {
    return WEBRTC_VIDEO_CODEC_ERR_PARAMETER;
  }

  decoder_->Decode(
      input_image.data(), input_image.size(), input_image.Timestamp(),
      [this](rtc::scoped_refptr<webrtc::VideoFrameBuffer> buffer,
             int64_t timestamp_rtp) {
        webrtc::VideoFrame decoded_image = webrtc::VideoFrame::Builder()
                                               .set_video_frame_buffer(buffer)
                                               .set_timestamp_rtp(timestamp_rtp)
                                               .build();
        decode_complete_callback_->Decoded(decoded_image, absl::nullopt,
                                           absl::nullopt);
      });

  return WEBRTC_VIDEO_CODEC_OK;
}

int32_t V4L2H264Decoder::RegisterDecodeCompleteCallback(
    webrtc::DecodedImageCallback* callback) {
  decode_complete_callback_ = callback;
  return WEBRTC_VIDEO_CODEC_OK;
}

int32_t V4L2H264Decoder::Release() {
  return WEBRTC_VIDEO_CODEC_OK;
}

const char* V4L2H264Decoder::ImplementationName() const {
  return "V4L2 Video";
}
