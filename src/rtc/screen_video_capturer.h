#ifndef SCREEN_VIDEO_CAPTURER_H_
#define SCREEN_VIDEO_CAPTURER_H_

#include <memory>
#include <vector>

// WebRTC
#include <api/scoped_refptr.h>
#include <modules/desktop_capture/desktop_capturer.h>
#include <modules/video_capture/video_capture.h>
#include <rtc_base/platform_thread.h>

#include "sora/scalable_track_source.h"

class ScreenVideoCapturer : public sora::ScalableVideoTrackSource,
                            public webrtc::DesktopCapturer::Callback {
 public:
  static bool GetSourceList(webrtc::DesktopCapturer::SourceList* sources);
  static const std::string GetSourceListString();
  ScreenVideoCapturer(webrtc::DesktopCapturer::SourceId source_id,
                      size_t max_width,
                      size_t max_height,
                      size_t target_fps);
  ~ScreenVideoCapturer();

 private:
  static void CaptureThread(void* obj);
  bool CaptureProcess();
  static webrtc::DesktopCaptureOptions CreateDesktopCaptureOptions();
  void OnCaptureResult(webrtc::DesktopCapturer::Result result,
                       std::unique_ptr<webrtc::DesktopFrame> frame) override;

  size_t max_width_;
  size_t max_height_;
  size_t capture_width_;
  size_t capture_height_;
  int requested_frame_duration_;
  int max_cpu_consumption_percentage_;
  webrtc::DesktopSize previous_frame_size_;
  std::unique_ptr<webrtc::DesktopFrame> output_frame_;
  rtc::PlatformThread capture_thread_;
  std::unique_ptr<webrtc::DesktopCapturer> capturer_;
  std::atomic<bool> quit_;
};

#endif  // SCREEN_VIDEO_CAPTURER_H_
