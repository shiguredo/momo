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

namespace sora {

struct V4L2VideoCapturerConfig : ScalableVideoTrackSourceConfig {
  std::string video_device;
  int width = 640;
  int height = 480;
  int framerate = 30;
  bool force_i420 = false;
  bool force_yuy2 = false;
  bool use_native = false;
  bool use_dmabuf = false;      // DMABUF モードを使用
  std::vector<int> dmabuf_fds;  // VPL からの DMABUF fd リスト
};

class V4L2VideoCapturer : public ScalableVideoTrackSource {
 public:
  static webrtc::scoped_refptr<V4L2VideoCapturer> Create(
      const V4L2VideoCapturerConfig& config);
  static void LogDeviceList(
      webrtc::VideoCaptureModule::DeviceInfo* device_info);
  V4L2VideoCapturer(const V4L2VideoCapturerConfig& config);
  ~V4L2VideoCapturer();

  int32_t Init(const char* deviceUniqueId);
  virtual int32_t StartCapture(const V4L2VideoCapturerConfig& config);

 protected:
  virtual int32_t StopCapture();
  virtual bool AllocateVideoBuffers();
  virtual bool DeAllocateVideoBuffers();
  virtual void OnCaptured(uint8_t* data, uint32_t bytesused);
  virtual bool AllocateDmaBufVideoBuffers(const std::vector<int>& dmabuf_fds);
  virtual bool DeAllocateDmaBufVideoBuffers();

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
  static webrtc::scoped_refptr<V4L2VideoCapturer> Create(
      webrtc::VideoCaptureModule::DeviceInfo* device_info,
      const V4L2VideoCapturerConfig& config,
      size_t capture_device_index);
  bool FindDevice(const char* deviceUniqueIdUTF8, const std::string& device);

  enum { kNoOfV4L2Bufffers = 4 };

  static void CaptureThread(void*);
  bool CaptureProcess();
  bool CaptureProcessDmaBuf();  // DMABUF モード専用のキャプチャ処理

  webrtc::PlatformThread _captureThread;
  webrtc::Mutex capture_lock_;
  bool quit_ RTC_GUARDED_BY(capture_lock_);
  std::string _videoDevice;

  int32_t _buffersAllocatedByDevice;
  bool _useNative;
  bool _captureStarted;
  bool _useDmaBuf;
  std::vector<int> _dmaBufFds;

  // DMABUF モード用のコールバック
  std::function<void(int buffer_index)> _dmaBufCallback;

 public:
  // DMABUF モードでバッファが準備できたときのコールバックを設定
  void SetDmaBufCallback(std::function<void(int buffer_index)> callback) {
    _dmaBufCallback = callback;
  }
};

}  // namespace sora

#endif
