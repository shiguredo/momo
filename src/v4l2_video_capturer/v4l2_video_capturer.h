/*
 *  Copyright (c) 2012 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef V4L2_VIDEO_CAPTURER_H_
#define V4L2_VIDEO_CAPTURER_H_

#include <stddef.h>
#include <stdint.h>

#include <memory>

// WebRTC
#include <modules/video_capture/video_capture_defines.h>
#include <modules/video_capture/video_capture_impl.h>
#include <rtc/scalable_track_source.h>
#include <rtc_base/platform_thread.h>
#include <rtc_base/synchronization/mutex.h>

struct V4L2VideoCapturerConfig {
  std::string video_device;
  int width = 640;
  int height = 480;
  int framerate = 30;
  bool force_i420 = false;
  bool use_native = false;
};

class V4L2VideoCapturer : public ScalableVideoTrackSource {
 public:
  static rtc::scoped_refptr<V4L2VideoCapturer> Create(
      V4L2VideoCapturerConfig config);
  static void LogDeviceList(
      webrtc::VideoCaptureModule::DeviceInfo* device_info);
  V4L2VideoCapturer();
  ~V4L2VideoCapturer();

  int32_t Init(const char* deviceUniqueId,
               const std::string& specifiedVideoDevice);
  virtual int32_t StartCapture(V4L2VideoCapturerConfig config);
  virtual bool UseNativeBuffer() override;

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
  static rtc::scoped_refptr<V4L2VideoCapturer> Create(
      webrtc::VideoCaptureModule::DeviceInfo* device_info,
      V4L2VideoCapturerConfig config,
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
};

#endif  // V4L2_VIDEO_CAPTURER_H_
