/*
 *  Copyright (c) 2012 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef JETSON_V4L2_CAPTURER_H_
#define JETSON_V4L2_CAPTURER_H_

#include <stddef.h>
#include <stdint.h>

#include <memory>

// WebRTC
#include <connection_settings.h>
#include <modules/video_capture/video_capture_defines.h>
#include <modules/video_capture/video_capture_impl.h>
#include <rtc/scalable_track_source.h>
#include <rtc_base/critical_section.h>
#include <rtc_base/platform_thread.h>

class JetsonV4L2Capturer : public ScalableVideoTrackSource {
 public:
  static rtc::scoped_refptr<JetsonV4L2Capturer> Create(ConnectionSettings cs);
  static void LogDeviceList(
      webrtc::VideoCaptureModule::DeviceInfo* device_info);
  JetsonV4L2Capturer();
  ~JetsonV4L2Capturer();

  int32_t Init(const char* deviceUniqueId,
               const std::string& specifiedVideoDevice);
  int32_t StartCapture(ConnectionSettings cs);
  int32_t StopCapture();
  bool UseNativeBuffer() override;
  bool OnCaptured(struct v4l2_buffer& buf);

 private:
  static rtc::scoped_refptr<JetsonV4L2Capturer> Create(
      webrtc::VideoCaptureModule::DeviceInfo* device_info,
      ConnectionSettings cs,
      size_t capture_device_index);
  bool FindDevice(const char* deviceUniqueIdUTF8, const std::string& device);

  enum { kNoOfV4L2Bufffers = 4 };

  bool AllocateVideoBuffers();
  bool DeAllocateVideoBuffers();
  static void CaptureThread(void*);
  bool CaptureProcess();

  // TODO(pbos): Stop using unique_ptr and resetting the thread.
  std::unique_ptr<rtc::PlatformThread> _captureThread;
  rtc::CriticalSection _captureCritSect;
  bool quit_ RTC_GUARDED_BY(_captureCritSect);
  std::string _videoDevice;

  int32_t _deviceFd;
  int32_t _buffersAllocatedByDevice;
  int32_t _currentWidth;
  int32_t _currentHeight;
  int32_t _currentFrameRate;
  bool _useNative;
  bool _captureStarted;
  webrtc::VideoType _captureVideoType;

  struct Buffer {
    void* start;
    size_t length;
    int dmabuff_fd;
  };
  Buffer* _pool;
};

#endif  // JETSON_V4L2_CAPTURER_H_
