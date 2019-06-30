/*
 *  Copyright (c) 2013 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */
#ifndef SCALABLE_VIDEO_TRACK_SOURCE_H_
#define SCALABLE_VIDEO_TRACK_SOURCE_H_

#include <stddef.h>

#include <memory>

#include "media/base/adapted_video_track_source.h"
#include "media/base/video_adapter.h"
#include "rtc_base/timestamp_aligner.h"

class ScalableVideoTrackSource : public rtc::AdaptedVideoTrackSource {
 public:
  ScalableVideoTrackSource();
  virtual ~ScalableVideoTrackSource();

  bool is_screencast() const override;
  absl::optional<bool> needs_denoising() const override;
  webrtc::MediaSourceInterface::SourceState state() const override;
  bool remote() const override;
  void OnCapturedFrame(const webrtc::VideoFrame& frame);

 private:
  rtc::TimestampAligner timestamp_aligner_;

  cricket::VideoAdapter video_adapter_;
};

#endif  // VIDEO_CAPTURER_H_