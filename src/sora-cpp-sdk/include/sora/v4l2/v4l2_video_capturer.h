/*
 *  Copyright (c) 2012 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef SORA_V4L2_VIDEO_CAPTURER_H_
#define SORA_V4L2_VIDEO_CAPTURER_H_

#include <stddef.h>
#include <stdint.h>
#include <string>

// WebRTC
#include <api/scoped_refptr.h>
#include <common_video/libyuv/include/webrtc_libyuv.h>
#include <modules/video_capture/video_capture.h>
#include <rtc_base/platform_thread.h>
#include <rtc_base/synchronization/mutex.h>
#include <rtc_base/thread_annotations.h>

#include "sora/scalable_track_source.h"
#include "sora/v4l2/v4l2_device.h"

namespace sora {

struct V4L2VideoCapturerConfig : ScalableVideoTrackSourceConfig {
  std::string video_device;
  int width = 640;
  int height = 480;
  int framerate = 30;
  bool force_i420 = false;
  bool force_yuy2 = false;
  bool force_nv12 = false;
  bool use_native = false;
};

class V4L2VideoCapturer : public ScalableVideoTrackSource {
 public:
  static webrtc::scoped_refptr<V4L2VideoCapturer> Create(
      const V4L2VideoCapturerConfig& config);
  V4L2VideoCapturer(const V4L2VideoCapturerConfig& config);
  ~V4L2VideoCapturer();

 protected:
  virtual int32_t StopCapture();
  virtual bool AllocateVideoBuffers();
  virtual bool DeAllocateVideoBuffers();
  virtual void OnCaptured(uint8_t* data, uint32_t bytesused);

  int32_t _deviceFd;
  int32_t _currentWidth;
  int32_t _currentHeight;
  int32_t _currentFrameRate;
  webrtc::VideoType _captureVideoType;
  struct Buffer {
    void* start;
    size_t length;
  };
  Buffer* _pool;

 private:
  int32_t Init();
  virtual int32_t StartCapture();

  enum { kNoOfV4L2Bufffers = 4 };

  static void CaptureThread(void*);
  bool CaptureProcess();

 private:
  V4L2VideoCapturerConfig config_;
  V4L2Device device_;

  webrtc::PlatformThread _captureThread;
  webrtc::Mutex capture_lock_;
  bool quit_ RTC_GUARDED_BY(capture_lock_);
  std::string _videoDevice;

  int32_t _buffersAllocatedByDevice;
  bool _useNative;
  bool _captureStarted;
};

}  // namespace sora

#endif
