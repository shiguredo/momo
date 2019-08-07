/*
 *  Copyright (c) 2012 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef V4L2_VIDEO_CAPTURE_H_
#define V4L2_VIDEO_CAPTURE_H_

#include <stddef.h>
#include <stdint.h>
#include <memory>

#include "modules/video_capture/video_capture_defines.h"
#include "modules/video_capture/video_capture_impl.h"
#include "rtc_base/critical_section.h"
#include "rtc_base/platform_thread.h"

#include "connection_settings.h"
#include "rtc/scalable_track_source.h"

class V4L2VideoCapture : public ScalableVideoTrackSource {
 public:
  static rtc::scoped_refptr<V4L2VideoCapture> Create(ConnectionSettings cs);
  static rtc::scoped_refptr<V4L2VideoCapture> Create(
      webrtc::VideoCaptureModule::DeviceInfo* device_info,
      ConnectionSettings cs,
      size_t capture_device_index);
  V4L2VideoCapture();
  ~V4L2VideoCapture();
  int32_t Init(const char* deviceUniqueId);
  int32_t StartCapture(ConnectionSettings cs);

  bool useNativeBuffer() override;

 private:
  enum { kNoOfV4L2Bufffers = 4 };

  int32_t StopCapture();
  bool AllocateVideoBuffers();
  bool DeAllocateVideoBuffers();
  static void CaptureThread(void*);
  bool CaptureProcess();

  // TODO(pbos): Stop using unique_ptr and resetting the thread.
  std::unique_ptr<rtc::PlatformThread> _captureThread;
  rtc::CriticalSection _captureCritSect;
  bool quit_ RTC_GUARDED_BY(_captureCritSect);
  int32_t _deviceId;
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
  };
  Buffer* _pool;
};

#endif  // V4L2_VIDEO_CAPTURE_H_
                                                                                
