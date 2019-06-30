/*
 *  Copyright (c) 2019 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */
#ifndef MAC_CAPTURER_H_
#define MAC_CAPTURER_H_

#include <memory>
#include <vector>

#include "api/media_stream_interface.h"
#include "api/scoped_refptr.h"
#include "modules/video_capture/video_capture.h"
#include "rtc_base/thread.h"

#include "rtc/scalable_track_source.h"

class MacCapturer : public ScalableVideoTrackSource,
                    public rtc::VideoSinkInterface<webrtc::VideoFrame> {
 public:
  static rtc::scoped_refptr<MacCapturer> Create(size_t width,
                             size_t height,
                             size_t target_fps,
                             size_t capture_device_index);
  MacCapturer(size_t width,
              size_t height,
              size_t target_fps,
              size_t capture_device_index);
  virtual ~MacCapturer();

  void OnFrame(const webrtc::VideoFrame& frame) override;

 private:
  void Destroy();

  void* capturer_;
  void* adapter_;
};

#endif  // TEST_MAC_CAPTURER_H_