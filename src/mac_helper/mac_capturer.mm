/*
 *  Copyright (c) 2019 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "mac_capturer.h"

#import "sdk/objc/base/RTCVideoCapturer.h"
#import "sdk/objc/components/capturer/RTCCameraVideoCapturer.h"
#import "sdk/objc/native/api/video_capturer.h"
#import "sdk/objc/native/src/objc_frame_buffer.h"

@interface RTCVideoSourceAdapter : NSObject <RTCVideoCapturerDelegate>
@property(nonatomic) MacCapturer *capturer;
@end

@implementation RTCVideoSourceAdapter
@synthesize capturer = _capturer;

- (void)capturer:(RTCVideoCapturer *)capturer didCaptureVideoFrame:(RTCVideoFrame *)frame {
  const int64_t timestamp_us = frame.timeStampNs / rtc::kNumNanosecsPerMicrosec;
  rtc::scoped_refptr<webrtc::VideoFrameBuffer> buffer =
      new rtc::RefCountedObject<webrtc::ObjCFrameBuffer>(frame.buffer);
  _capturer->OnFrame(webrtc::VideoFrame::Builder()
                         .set_video_frame_buffer(buffer)
                         .set_rotation(webrtc::kVideoRotation_0)
                         .set_timestamp_us(timestamp_us)
                         .build());
}

@end

namespace {

AVCaptureDeviceFormat *SelectClosestFormat(AVCaptureDevice *device, size_t width, size_t height) {
  NSArray<AVCaptureDeviceFormat *> *formats =
      [RTCCameraVideoCapturer supportedFormatsForDevice:device];
  AVCaptureDeviceFormat *selectedFormat = nil;
  int currentDiff = INT_MAX;
  for (AVCaptureDeviceFormat *format in formats) {
    CMVideoDimensions dimension = CMVideoFormatDescriptionGetDimensions(format.formatDescription);
    int diff =
        std::abs((int64_t)width - dimension.width) + std::abs((int64_t)height - dimension.height);
    if (diff < currentDiff) {
      selectedFormat = format;
      currentDiff = diff;
    }
  }
  return selectedFormat;
}

}  // namespace

MacCapturer::MacCapturer(size_t width,
                         size_t height,
                         size_t target_fps,
                         size_t capture_device_index) {
  RTCVideoSourceAdapter *adapter = [[RTCVideoSourceAdapter alloc] init];
  adapter_ = (__bridge_retained void *)adapter;
  adapter.capturer = this;

  RTCCameraVideoCapturer *capturer = [[RTCCameraVideoCapturer alloc] initWithDelegate:adapter];
  capturer_ = (__bridge_retained void *)capturer;

  AVCaptureDevice *device =
      [[RTCCameraVideoCapturer captureDevices] objectAtIndex:capture_device_index];
  AVCaptureDeviceFormat *format = SelectClosestFormat(device, width, height);
  [capturer startCaptureWithDevice:device format:format fps:target_fps];
}

rtc::scoped_refptr<MacCapturer> MacCapturer::Create(size_t width,
                                 size_t height,
                                 size_t target_fps,
                                 size_t capture_device_index) {
  return new rtc::RefCountedObject<MacCapturer>(width, height, target_fps, capture_device_index);
}

void MacCapturer::Destroy() {
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunused-variable"
  RTCVideoSourceAdapter *adapter = (__bridge_transfer RTCVideoSourceAdapter *)adapter_;
  RTCCameraVideoCapturer *capturer = (__bridge_transfer RTCCameraVideoCapturer *)capturer_;
  [capturer stopCapture];
#pragma clang diagnostic pop
}

MacCapturer::~MacCapturer() {
  Destroy();
}

void MacCapturer::OnFrame(const webrtc::VideoFrame &frame) {
  OnCapturedFrame(frame);
}