/*
 *  Copyright 2012 The WebRTC Project Authors. All rights reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef CAPTURE_TRACK_SOURCE_H_
#define CAPTURE_TRACK_SOURCE_H_

#include "pc/video_track_source.h"
#include "rtc_base/logging.h"

class CapturerTrackSource : public webrtc::VideoTrackSource {
 public:
  static rtc::scoped_refptr<CapturerTrackSource> Create(
    std::unique_ptr<rtc::VideoSourceInterface<webrtc::VideoFrame>> capturer) {
    return new rtc::RefCountedObject<CapturerTrackSource>(std::move(capturer));
  }

 protected:
  explicit CapturerTrackSource(
      std::unique_ptr<rtc::VideoSourceInterface<webrtc::VideoFrame>> capturer)
      : VideoTrackSource(/*remote=*/false), capturer_(std::move(capturer)) {}

 private:
  rtc::VideoSourceInterface<webrtc::VideoFrame>* source() override {
    return capturer_.get();
  }
  std::unique_ptr<rtc::VideoSourceInterface<webrtc::VideoFrame>> capturer_;
};

#endif