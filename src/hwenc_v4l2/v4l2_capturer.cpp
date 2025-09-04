#include "v4l2_capturer.h"

// Linux
#include <sys/ioctl.h>

// WebRTC
#include <api/video/nv12_buffer.h>
#include <modules/video_capture/video_capture_factory.h>
#include <rtc_base/logging.h>

#include "v4l2_native_buffer.h"

webrtc::scoped_refptr<V4L2Capturer> V4L2Capturer::Create(
    sora::V4L2VideoCapturerConfig config) {
  return webrtc::make_ref_counted<V4L2Capturer>(config);
}

V4L2Capturer::V4L2Capturer(const sora::V4L2VideoCapturerConfig& config)
    : sora::V4L2VideoCapturer(config) {}

void V4L2Capturer::OnCaptured(uint8_t* data, uint32_t bytesused) {
  const int64_t timestamp_us = webrtc::TimeMicros();
  int adapted_width, adapted_height, crop_width, crop_height, crop_x, crop_y;
  if (!AdaptFrame(_currentWidth, _currentHeight, timestamp_us, &adapted_width,
                  &adapted_height, &crop_width, &crop_height, &crop_x,
                  &crop_y)) {
    return;
  }

  auto frame_buffer = webrtc::make_ref_counted<V4L2NativeBuffer>(
      webrtc::VideoType::kMJPEG, _currentWidth, _currentHeight, adapted_width,
      adapted_height, 0, data, bytesused, _currentWidth, nullptr);

  webrtc::VideoFrame video_frame = webrtc::VideoFrame::Builder()
                                       .set_video_frame_buffer(frame_buffer)
                                       .set_timestamp_rtp(0)
                                       .set_timestamp_ms(webrtc::TimeMillis())
                                       .set_timestamp_us(webrtc::TimeMicros())
                                       .set_rotation(webrtc::kVideoRotation_0)
                                       .build();
  OnFrame(video_frame);
}