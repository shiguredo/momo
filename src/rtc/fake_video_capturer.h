#ifndef RTC_FAKE_VIDEO_CAPTURER_H_
#define RTC_FAKE_VIDEO_CAPTURER_H_

#if defined(USE_FAKE_CAPTURE_DEVICE)

#include <atomic>
#include <chrono>
#include <memory>
#include <thread>

// WebRTC
#include <api/video/i420_buffer.h>
#include <rtc_base/ref_counted_object.h>

// Sora C++ SDK
#include <sora/scalable_track_source.h>

// Blend2D
#include <blend2d.h>

// Forward declaration
class FakeAudioCapturer;

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

  FakeVideoCapturer(Config config);
  ~FakeVideoCapturer();
  void StartCapture();
  void StopCapture();

  void SetAudioCapturer(
      webrtc::scoped_refptr<FakeAudioCapturer> audio_capturer);

 private:
  webrtc::scoped_refptr<FakeAudioCapturer> GetAudioCapturer() const;

  void CaptureThread();
  void UpdateImage(std::chrono::high_resolution_clock::time_point now);
  void DrawAnimations(BLContext& ctx,
                      std::chrono::high_resolution_clock::time_point now);
  void DrawBoxes(BLContext& ctx,
                 std::chrono::high_resolution_clock::time_point now);
  void DrawDigitalClock(BLContext& ctx,
                        std::chrono::high_resolution_clock::time_point now);
  void Draw7Segment(BLContext& ctx,
                    int digit,
                    double x,
                    double y,
                    double width,
                    double height);
  void DrawColon(BLContext& ctx, double x, double y, double height);

  Config config_;
  std::unique_ptr<std::thread> capture_thread_;
  std::atomic<bool> stop_capture_{false};
  std::chrono::high_resolution_clock::time_point start_time_;

  // Blend2D 関連
  BLImage image_;
  std::atomic<uint32_t> frame_counter_{0};

  // エラーハンドリング用
  static constexpr int kMaxConsecutiveErrors = 10;
  int consecutive_error_count_ = 0;

  mutable std::mutex audio_capturer_mutex_;
  webrtc::scoped_refptr<FakeAudioCapturer> audio_capturer_;
};

#endif  // USE_FAKE_CAPTURE_DEVICE

#endif  // RTC_FAKE_VIDEO_CAPTURER_H_