#ifndef JETSON_V4L2_CAPTURER_H_
#define JETSON_V4L2_CAPTURER_H_

#include <v4l2_video_capturer/v4l2_video_capturer.h>

#include "jetson_jpeg_decoder_pool.h"

class JetsonV4L2Capturer : public V4L2VideoCapturer {
 public:
  static rtc::scoped_refptr<V4L2VideoCapturer> Create(V4L2VideoCapturerConfig config);

  bool UseNativeBuffer() override;
 private:
  static rtc::scoped_refptr<V4L2VideoCapturer> Create(
      webrtc::VideoCaptureModule::DeviceInfo* device_info,
      V4L2VideoCapturerConfig config,
      size_t capture_device_index);

  bool AllocateVideoBuffers() override;
  bool DeAllocateVideoBuffers() override;
  bool OnCaptured(struct v4l2_buffer& buf) override;

  std::shared_ptr<JetsonJpegDecoderPool> jpeg_decoder_pool_;
};

#endif  // JETSON_V4L2_CAPTURER_H_
