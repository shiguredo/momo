/*
 *  Copyright (c) 2017 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "scalable_track_source.h"

#include <algorithm>

#include "api/scoped_refptr.h"
#include "api/video/i420_buffer.h"
#include "api/video/video_frame_buffer.h"
#include "api/video/video_rotation.h"
#include "native_buffer.h"
#include "rtc_base/logging.h"

ScalableVideoTrackSource::ScalableVideoTrackSource()
    : AdaptedVideoTrackSource(4) {}
ScalableVideoTrackSource::~ScalableVideoTrackSource() {}

bool ScalableVideoTrackSource::is_screencast() const {
  return false;
}

absl::optional<bool> ScalableVideoTrackSource::needs_denoising() const {
  return false;
}

webrtc::MediaSourceInterface::SourceState ScalableVideoTrackSource::state()
    const {
  return SourceState::kLive;
}

bool ScalableVideoTrackSource::remote() const {
  return false;
}

void ScalableVideoTrackSource::OnCapturedFrame(
    const webrtc::VideoFrame& frame) {
  const int64_t timestamp_us = frame.timestamp_us();
  const int64_t translated_timestamp_us =
      timestamp_aligner_.TranslateTimestamp(timestamp_us, rtc::TimeMicros());

  int adapted_width;
  int adapted_height;
  int crop_width;
  int crop_height;
  int crop_x;
  int crop_y;
  if (!AdaptFrame(frame.width(), frame.height(), timestamp_us, &adapted_width,
                  &adapted_height, &crop_width, &crop_height, &crop_x,
                  &crop_y)) {
    return;
  }

  if (useNativeBuffer() && frame.video_frame_buffer()->type() ==
                               webrtc::VideoFrameBuffer::Type::kNative) {
    NativeBuffer* frame_buffer =
        dynamic_cast<NativeBuffer*>(frame.video_frame_buffer().get());
    frame_buffer->SetScaledSize(adapted_width, adapted_height);
    OnFrame(frame);
    return;
  }

  rtc::scoped_refptr<webrtc::VideoFrameBuffer> buffer =
      frame.video_frame_buffer();

  if (adapted_width != frame.width() || adapted_height != frame.height()) {
    // Video adapter has requested a down-scale. Allocate a new buffer and
    // return scaled version.
    rtc::scoped_refptr<webrtc::I420Buffer> i420_buffer =
        webrtc::I420Buffer::Create(adapted_width, adapted_height);
    i420_buffer->ScaleFrom(*buffer->ToI420());
    buffer = i420_buffer;
  }

  OnFrame(webrtc::VideoFrame::Builder()
              .set_video_frame_buffer(buffer)
              .set_rotation(frame.rotation())
              .set_timestamp_us(translated_timestamp_us)
              .build());
}
