/*
 *  Copyright (c) 2013 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */
#ifndef DEVICE_VIDEO_CAPTURER_H_
#define DEVICE_VIDEO_CAPTURER_H_

#include <memory>
#include <vector>

#include "api/scoped_refptr.h"
#include "modules/video_capture/video_capture.h"

#include "video_capturer.h"


class DeviceVideoCapturer : public VideoCapturer,
                    public rtc::VideoSinkInterface<webrtc::VideoFrame> {
 public:
  static std::unique_ptr<DeviceVideoCapturer> Create(size_t width,
                             size_t height,
                             size_t target_fps);
  static DeviceVideoCapturer* Create(size_t width,
                             size_t height,
                             size_t target_fps,
                             size_t capture_device_index);
  virtual ~DeviceVideoCapturer();

  void OnFrame(const webrtc::VideoFrame& frame) override;

 private:
  DeviceVideoCapturer();
  bool Init(size_t width,
            size_t height,
            size_t target_fps,
            size_t capture_device_index);
  void Destroy();

  rtc::scoped_refptr<webrtc::VideoCaptureModule> vcm_;
  webrtc::VideoCaptureCapability capability_;
};

#endif  // DEVICE_VIDEO_CAPTURER_H_