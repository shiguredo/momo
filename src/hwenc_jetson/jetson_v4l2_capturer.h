#ifndef JETSON_V4L2_CAPTURER_H_
#define JETSON_V4L2_CAPTURER_H_

#include <v4l2_video_capturer/v4l2_video_capturer.h>

class JetsonV4L2Capturer : public V4L2VideoCapturer {
 public:
  static rtc::scoped_refptr<V4L2VideoCapturer> Create(ConnectionSettings cs);

  bool UseNativeBuffer() override;
 private:
  static rtc::scoped_refptr<V4L2VideoCapturer> Create(
      webrtc::VideoCaptureModule::DeviceInfo* device_info,
      ConnectionSettings cs,
      size_t capture_device_index);
  bool OnCaptured(struct v4l2_buffer& buf) override;
};

#endif  // JETSON_V4L2_CAPTURER_H_
