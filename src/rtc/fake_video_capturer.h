#ifndef RTC_FAKE_VIDEO_CAPTURER_H_
#define RTC_FAKE_VIDEO_CAPTURER_H_

#include <atomic>
#include <chrono>
#include <memory>
#include <thread>

// WebRTC
#include <api/video/i420_buffer.h>
#include <rtc_base/ref_counted_object.h>

// Sora C++ SDK
#include <sora/scalable_track_source.h>

#if defined(USE_FAKE_CAPTURE_DEVICE)
// Blend2D
#include <blend2d.h>

class FakeVideoCapturer : public sora::ScalableVideoTrackSource {
 public:
  struct Config : sora::ScalableVideoTrackSourceConfig {
    int width = 640;
    int height = 480;
    int fps = 30;
  };

  static webrtc::scoped_refptr<FakeVideoCapturer> Create(Config config) {
    return webrtc::make_ref_counted<FakeVideoCapturer>(std::move(config));
  }

  ~FakeVideoCapturer();
  void StartCapture();
  void StopCapture();

 protected:
  explicit FakeVideoCapturer(Config config);

 private:
  void CaptureThread();
  void UpdateImage(std::chrono::high_resolution_clock::time_point now);
  void DrawAnimations(BLContext& ctx,
                      std::chrono::high_resolution_clock::time_point now);
  void DrawBoxes(BLContext& ctx,
                 std::chrono::high_resolution_clock::time_point now);
  void DrawDigitalClock(BLContext& ctx,
                        std::chrono::high_resolution_clock::time_point now);
  void Draw7Segment(BLContext& ctx, int digit, double x, double y, 
                    double width, double height);
  void DrawColon(BLContext& ctx, double x, double y, double height);

  Config config_;
  std::unique_ptr<std::thread> capture_thread_;
  std::atomic<bool> stop_capture_{false};
  std::chrono::high_resolution_clock::time_point start_time_;
  
  // Blend2D 関連
  BLImage image_;
  uint32_t frame_counter_ = 0;

  friend class webrtc::RefCountedObject<FakeVideoCapturer>;
};

#else  // USE_FAKE_CAPTURE_DEVICE

// Blend2D が無い場合のダミー実装
class FakeVideoCapturer : public sora::ScalableVideoTrackSource {
 public:
  struct Config : sora::ScalableVideoTrackSourceConfig {
    int width = 640;
    int height = 480;
    int fps = 30;
  };

  static webrtc::scoped_refptr<FakeVideoCapturer> Create(Config config) {
    return webrtc::make_ref_counted<FakeVideoCapturer>(std::move(config));
  }

  ~FakeVideoCapturer() {}

 protected:
  explicit FakeVideoCapturer(Config config)
      : sora::ScalableVideoTrackSource(config) {}

  friend class webrtc::RefCountedObject<FakeVideoCapturer>;
};

#endif  // USE_FAKE_CAPTURE_DEVICE

#endif  // RTC_FAKE_VIDEO_CAPTURER_H_