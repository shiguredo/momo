/*
 *  Copyright (c) 2017 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "sora/scalable_track_source.h"

#include <cstdint>
#include <optional>

// WebRTC
#include <api/media_stream_interface.h>
#include <api/scoped_refptr.h>
#include <api/video/i420_buffer.h>
#include <api/video/video_frame.h>
#include <api/video/video_frame_buffer.h>
#include <api/video/video_rotation.h>
#include <media/base/adapted_video_track_source.h>
#include <rtc_base/time_utils.h>

// libyuv
#include <libyuv/rotate.h>

namespace sora {

ScalableVideoTrackSource::ScalableVideoTrackSource(
    ScalableVideoTrackSourceConfig config)
    : AdaptedVideoTrackSource(4), config_(config) {}
ScalableVideoTrackSource::~ScalableVideoTrackSource() {}

bool ScalableVideoTrackSource::is_screencast() const {
  return false;
}

std::optional<bool> ScalableVideoTrackSource::needs_denoising() const {
  return false;
}

webrtc::MediaSourceInterface::SourceState ScalableVideoTrackSource::state()
    const {
  return SourceState::kLive;
}

bool ScalableVideoTrackSource::remote() const {
  return false;
}

bool ScalableVideoTrackSource::OnCapturedFrame(
    const webrtc::VideoFrame& video_frame) {
  webrtc::VideoFrame frame = video_frame;

  const int64_t timestamp_us = frame.timestamp_us();
  const int64_t translated_timestamp_us =
      timestamp_aligner_.TranslateTimestamp(timestamp_us, webrtc::TimeMicros());

  // 回転が必要
  if (frame.rotation() != webrtc::kVideoRotation_0) {
    int width;
    int height;
    libyuv::RotationMode mode;
    switch (frame.rotation()) {
      case webrtc::kVideoRotation_180:
        width = frame.width();
        height = frame.height();
        mode = libyuv::kRotate180;
        break;
      case webrtc::kVideoRotation_90:
        width = frame.height();
        height = frame.width();
        mode = libyuv::kRotate90;
        break;
      case webrtc::kVideoRotation_270:
      default:
        width = frame.height();
        height = frame.width();
        mode = libyuv::kRotate270;
        break;
    }

    webrtc::scoped_refptr<webrtc::I420Buffer> rotated =
        webrtc::I420Buffer::Create(width, height);
    webrtc::scoped_refptr<webrtc::I420BufferInterface> src =
        frame.video_frame_buffer()->ToI420();
    libyuv::I420Rotate(src->DataY(), src->StrideY(), src->DataU(),
                       src->StrideU(), src->DataV(), src->StrideV(),
                       rotated->MutableDataY(), rotated->StrideY(),
                       rotated->MutableDataU(), rotated->StrideU(),
                       rotated->MutableDataV(), rotated->StrideV(),
                       frame.width(), frame.height(), mode);
    frame.set_video_frame_buffer(rotated);
    frame.set_rotation(webrtc::kVideoRotation_0);
  }

  int adapted_width;
  int adapted_height;
  int crop_width;
  int crop_height;
  int crop_x;
  int crop_y;
  if (!AdaptFrame(frame.width(), frame.height(), timestamp_us, &adapted_width,
                  &adapted_height, &crop_width, &crop_height, &crop_x,
                  &crop_y)) {
    return false;
  }

  if (config_.on_frame) {
    config_.on_frame(frame);
  }

  if (frame.video_frame_buffer()->type() ==
      webrtc::VideoFrameBuffer::Type::kNative) {
    OnFrame(frame);
    return true;
  }

  webrtc::scoped_refptr<webrtc::VideoFrameBuffer> buffer =
      frame.video_frame_buffer();

  if (adapted_width != frame.width() || adapted_height != frame.height()) {
    // Video adapter has requested a down-scale. Allocate a new buffer and
    // return scaled version.
    webrtc::scoped_refptr<webrtc::I420Buffer> i420_buffer =
        webrtc::I420Buffer::Create(adapted_width, adapted_height);
    i420_buffer->ScaleFrom(*buffer->ToI420());
    buffer = i420_buffer;
  }

  OnFrame(webrtc::VideoFrame::Builder()
              .set_video_frame_buffer(buffer)
              .set_rotation(frame.rotation())
              .set_timestamp_us(translated_timestamp_us)
              .build());

  return true;
}

}  // namespace sora
