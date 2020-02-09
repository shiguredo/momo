#ifndef V4L2_MMAL_CAPTURE_H_
#define V4L2_MMAL_CAPTURE_H_

#include <stddef.h>
#include <stdint.h>

#include <memory>

#include "v4l2_video_capturer/v4l2_video_capturer.h"

class V4L2MMALCapture : public V4L2VideoCapture {
 public:
  static rtc::scoped_refptr<V4L2VideoCapture> Create(ConnectionSettings cs);

 protected:
  int32_t StartCapture(ConnectionSettings cs) override;
  int32_t StopCapture() override;
  bool useNativeBuffer() override;
  void OnCaptured(struct v4l2_buffer& buf) override;

 private:
  static rtc::scoped_refptr<V4L2VideoCapture> Create(
      webrtc::VideoCaptureModule::DeviceInfo* device_info,
      ConnectionSettings cs,
      size_t capture_device_index);
};

#endif  // V4L2_MMAL_CAPTURE_H_
