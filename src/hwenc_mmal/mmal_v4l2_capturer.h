#ifndef MMAL_V4L2_CAPTURER_H_
#define MMAL_V4L2_CAPTURER_H_

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

#include <stddef.h>
#include <stdint.h>

#include <memory>
#include <mutex>
#include <queue>

// WebRTC
#include <rtc_base/synchronization/mutex.h>

#include "v4l2_video_capturer/v4l2_video_capturer.h"

typedef V4L2VideoCapturerConfig MMALV4L2CapturerConfig;

class MMALV4L2Capturer : public V4L2VideoCapturer {
 public:
  static rtc::scoped_refptr<V4L2VideoCapturer> Create(
      MMALV4L2CapturerConfig config);
  MMALV4L2Capturer();
  ~MMALV4L2Capturer();

 private:
  struct FrameParams {
    FrameParams(int32_t w, int32_t h, int64_t ts)
        : width(w), height(h), timestamp(ts) {}

    int32_t width;
    int32_t height;
    int64_t timestamp;
  };

  static rtc::scoped_refptr<V4L2VideoCapturer> Create(
      webrtc::VideoCaptureModule::DeviceInfo* device_info,
      MMALV4L2CapturerConfig config,
      size_t capture_device_index);
  int32_t StartCapture(V4L2VideoCapturerConfig config) override;
  int32_t StopCapture() override;
  bool UseNativeBuffer() override;
  void OnCaptured(uint8_t* data, uint32_t bytesused) override;
  static void MMALInputCallbackFunction(MMAL_PORT_T* port,
                                        MMAL_BUFFER_HEADER_T* buffer);
  void MMALInputCallback(MMAL_PORT_T* port, MMAL_BUFFER_HEADER_T* buffer);
  void ResizerFillBuffer();
  static void ResizerOutputCallbackFunction(MMAL_PORT_T* port,
                                            MMAL_BUFFER_HEADER_T* buffer);
  void ResizerOutputCallback(MMAL_PORT_T* port, MMAL_BUFFER_HEADER_T* buffer);
  int32_t MMALConfigure(int32_t width, int32_t height);
  void MMALRelease();

  std::mutex mtx_;
  MMAL_COMPONENT_T* component_in_;
  MMAL_COMPONENT_T* decoder_;
  MMAL_COMPONENT_T* resizer_;
  MMAL_CONNECTION_T* connection_;
  MMAL_POOL_T* pool_in_;
  MMAL_POOL_T* resizer_pool_out_;
  int32_t configured_width_;
  int32_t configured_height_;
  webrtc::Mutex frame_params_lock_;
  std::queue<std::unique_ptr<FrameParams>> frame_params_;
  unsigned int decoded_buffer_num_;
  size_t decoded_buffer_size_;
};

#endif  // MMAL_V4L2_CAPTURER_H_
