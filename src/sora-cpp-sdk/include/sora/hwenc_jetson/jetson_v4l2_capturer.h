#ifndef SORA_HWENC_JETSON_JETSON_V4L2_CAPTURER_H_
#define SORA_HWENC_JETSON_JETSON_V4L2_CAPTURER_H_

#include <stddef.h>
#include <stdint.h>

#include <memory>

// Linux
#include <linux/videodev2.h>

// WebRTC
#include <modules/video_capture/video_capture_defines.h>
#include <modules/video_capture/video_capture_impl.h>
#include <rtc_base/platform_thread.h>
#include <rtc_base/synchronization/mutex.h>

#include "jetson_jpeg_decoder_pool.h"
#include "sora/scalable_track_source.h"
#include "sora/v4l2/v4l2_video_capturer.h"

namespace sora {

class JetsonV4L2Capturer : public ScalableVideoTrackSource {
 public:
  static rtc::scoped_refptr<JetsonV4L2Capturer> Create(
      const V4L2VideoCapturerConfig& config);
  JetsonV4L2Capturer(const V4L2VideoCapturerConfig& config);
  ~JetsonV4L2Capturer();

 private:
  static void LogDeviceList(
      webrtc::VideoCaptureModule::DeviceInfo* device_info);

  int32_t Init(const char* deviceUniqueId,
               const std::string& specifiedVideoDevice);
  int32_t StartCapture(const V4L2VideoCapturerConfig& config);

  int32_t StopCapture();
  bool AllocateVideoBuffers();
  bool DeAllocateVideoBuffers();
  void OnCaptured(v4l2_buffer* buf);

  int32_t _deviceFd;
  int32_t _currentWidth;
  int32_t _currentHeight;
  int32_t _currentPixelFormat;
  int32_t _currentFrameRate;
  webrtc::VideoType _captureVideoType;
  struct Buffer {
    void* start;
    size_t length;
    int fd;
  };
  Buffer* _pool;

 private:
  static rtc::scoped_refptr<JetsonV4L2Capturer> Create(
      webrtc::VideoCaptureModule::DeviceInfo* device_info,
      const V4L2VideoCapturerConfig& config,
      size_t capture_device_index);
  bool FindDevice(const char* deviceUniqueIdUTF8, const std::string& device);

  enum { kNoOfV4L2Bufffers = 4 };

  static void CaptureThread(void*);
  bool CaptureProcess();

  rtc::PlatformThread _captureThread;
  webrtc::Mutex capture_lock_;
  bool quit_ RTC_GUARDED_BY(capture_lock_);
  std::string _videoDevice;

  int32_t _buffersAllocatedByDevice;
  bool _useNative;
  bool _captureStarted;

  std::shared_ptr<JetsonJpegDecoderPool> jpeg_decoder_pool_;
};

}  // namespace sora

#endif
