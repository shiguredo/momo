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

// WebRTC
#include <api/scoped_refptr.h>
#include <modules/video_capture/video_capture.h>
#include <rtc_base/ref_counted_object.h>

#include "sora/scalable_track_source.h"

class DeviceVideoCapturer
    : public sora::ScalableVideoTrackSource,
      public webrtc::VideoSinkInterface<webrtc::VideoFrame> {
 public:
  static webrtc::scoped_refptr<DeviceVideoCapturer> Create(size_t width,
                                                           size_t height,
                                                           size_t target_fps);
  static webrtc::scoped_refptr<DeviceVideoCapturer> Create(
      size_t width,
      size_t height,
      size_t target_fps,
      size_t capture_device_index);
  static webrtc::scoped_refptr<DeviceVideoCapturer> Create(
      size_t width,
      size_t height,
      size_t target_fps,
      const std::string& capture_device);
  static webrtc::scoped_refptr<DeviceVideoCapturer> Create(
      size_t width,
      size_t height,
      size_t target_fps,
      const std::string& capture_device,
      bool force_yuy2);
  DeviceVideoCapturer();
  virtual ~DeviceVideoCapturer();

 private:
  bool Init(size_t width,
            size_t height,
            size_t target_fps,
            size_t capture_device_index);
  bool Init(size_t width,
            size_t height,
            size_t target_fps,
            size_t capture_device_index,
            bool force_yuy2);
  void Destroy();

  // webrtc::VideoSinkInterface interface.
  void OnFrame(const webrtc::VideoFrame& frame) override;

  int LogDeviceInfo();
  int GetDeviceIndex(const std::string& device);

  webrtc::scoped_refptr<webrtc::VideoCaptureModule> vcm_;
  webrtc::VideoCaptureCapability capability_;
  bool force_yuy2_ = false;
};

#endif  // DEVICE_VIDEO_CAPTURER_H_
