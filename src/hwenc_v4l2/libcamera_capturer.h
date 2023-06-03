#ifndef LIBCAMERA_CAPTURER_H_
#define LIBCAMERA_CAPTURER_H_

#include <memory>
#include <mutex>
#include <queue>

#include <v4l2_video_capturer/v4l2_video_capturer.h>

#include "libcamerac/libcameracpp.h"
#include "rtc/scalable_track_source.h"

class LibcameraCapturer : public ScalableVideoTrackSource {
 public:
  static rtc::scoped_refptr<LibcameraCapturer> Create(
      V4L2VideoCapturerConfig config);
  static void LogDeviceList();
  LibcameraCapturer();
  ~LibcameraCapturer();

  int32_t Init(int camera_id);
  void Release();
  int32_t StartCapture(V4L2VideoCapturerConfig config);

 private:
  static rtc::scoped_refptr<LibcameraCapturer> Create(
      V4L2VideoCapturerConfig config,
      size_t capture_device_index);
  int32_t StopCapture();
  static void requestCompleteStatic(libcamerac_Request* request,
                                    void* user_data);
  void requestComplete(libcamerac_Request* request);
  void queueRequest(libcamerac_Request* request);

  std::shared_ptr<libcamerac_CameraManager> camera_manager_;
  std::shared_ptr<libcamerac_Camera> camera_;
  bool acquired_;
  std::shared_ptr<libcamerac_CameraConfiguration> configuration_;
  libcamerac_Stream* stream_;
  std::shared_ptr<libcamerac_FrameBufferAllocator> allocator_ = nullptr;
  struct Span {
    uint8_t* buffer;
    int length;
  };
  std::map<const libcamerac_FrameBuffer*, std::vector<Span>> mapped_buffers_;
  std::queue<libcamerac_FrameBuffer*> frame_buffer_;
  std::vector<std::shared_ptr<libcamerac_Request>> requests_;
  std::shared_ptr<libcamerac_ControlList> controls_;
  bool camera_started_;
  std::mutex camera_stop_mutex_;
};

#endif  // LIBCAMERA_CAPTURER_H_