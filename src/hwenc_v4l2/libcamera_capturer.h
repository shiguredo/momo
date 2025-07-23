#ifndef LIBCAMERA_CAPTURER_H_
#define LIBCAMERA_CAPTURER_H_

#include <memory>
#include <mutex>
#include <queue>

#include "libcamerac/libcameracpp.h"
#include "sora/scalable_track_source.h"
#include "sora/v4l2/v4l2_video_capturer.h"

struct LibcameraCapturerConfig : sora::V4L2VideoCapturerConfig {
  LibcameraCapturerConfig() {}
  LibcameraCapturerConfig(const sora::V4L2VideoCapturerConfig& config) {
    *static_cast<sora::V4L2VideoCapturerConfig*>(this) = config;
  }
  LibcameraCapturerConfig(const LibcameraCapturerConfig& config) {
    *this = config;
  }
  // native_frame_output == true の場合、キャプチャしたデータを kNative なフレームとして渡す。
  // native_frame_output == false の場合、データをコピーして I420Buffer なフレームを作って渡す。
  // 前者の方が効率が良いけれども、kNative なフレームはサイマルキャスト時に自動で
  // リサイズしてくれないので、状況に応じて使い分けるのが良い。
  bool native_frame_output = false;
};

// Raspberry Pi 専用のカメラからの映像を取得するクラス
// 出力の形式として、fd そのままで取得する形式と、メモリ上にコピーして取得する形式がある
// 渡されるフレームバッファは、fd そのままで取得する場合は V4L2NativeBuffer クラスになり、
// メモリ上にコピーする場合は webrtc::I420Buffer クラスになる。
class LibcameraCapturer : public sora::ScalableVideoTrackSource {
 public:
  static webrtc::scoped_refptr<LibcameraCapturer> Create(
      LibcameraCapturerConfig config);
  static void LogDeviceList();
  LibcameraCapturer();
  ~LibcameraCapturer();

  int32_t Init(int camera_id);
  void Release();
  int32_t StartCapture(LibcameraCapturerConfig config);

 private:
  static webrtc::scoped_refptr<LibcameraCapturer> Create(
      LibcameraCapturerConfig config,
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
    int fd;
  };
  std::map<const libcamerac_FrameBuffer*, std::vector<Span>> mapped_buffers_;
  std::queue<libcamerac_FrameBuffer*> frame_buffer_;
  std::vector<std::shared_ptr<libcamerac_Request>> requests_;
  std::shared_ptr<libcamerac_ControlList> controls_;
  bool camera_started_;
  std::mutex camera_stop_mutex_;
};

#endif  // LIBCAMERA_CAPTURER_H_