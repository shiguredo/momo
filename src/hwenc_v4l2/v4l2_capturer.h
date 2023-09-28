#ifndef V4L2_CAPTURER_H_
#define V4L2_CAPTURER_H_

#include "../v4l2_video_capturer/v4l2_video_capturer.h"

class V4L2Capturer : public V4L2VideoCapturer {
 public:
  static rtc::scoped_refptr<V4L2VideoCapturer> Create(
      V4L2VideoCapturerConfig config);
  V4L2Capturer(const V4L2VideoCapturerConfig& config);

  bool UseNativeBuffer() override;

 private:
  static rtc::scoped_refptr<V4L2VideoCapturer> Create(
      webrtc::VideoCaptureModule::DeviceInfo* device_info,
      V4L2VideoCapturerConfig config,
      size_t capture_device_index);

  void OnCaptured(uint8_t* data, uint32_t bytesused) override;
};

#endif  // NVCODEC_V4L2_CAPTURER_H_
