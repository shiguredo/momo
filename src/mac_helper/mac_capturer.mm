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

#include <iostream>

// WebRTC
#include <rtc_base/logging.h>

// WebRTC
#import <sdk/objc/base/RTCVideoCapturer.h>
#import <sdk/objc/components/capturer/RTCCameraVideoCapturer.h>
#import <sdk/objc/native/api/video_capturer.h>
#import <sdk/objc/native/src/objc_frame_buffer.h>

@interface RTCVideoSourceAdapter : NSObject <RTCVideoCapturerDelegate>
@property(nonatomic) MacCapturer* capturer;
@end

@implementation RTCVideoSourceAdapter
@synthesize capturer = _capturer;

- (void)capturer:(RTCVideoCapturer*)capturer
    didCaptureVideoFrame:(RTCVideoFrame*)frame {
  const int64_t timestamp_us =
      frame.timeStampNs / webrtc::kNumNanosecsPerMicrosec;
  webrtc::scoped_refptr<webrtc::VideoFrameBuffer> buffer =
      webrtc::make_ref_counted<webrtc::ObjCFrameBuffer>(frame.buffer);
  _capturer->OnFrame(webrtc::VideoFrame::Builder()
                         .set_video_frame_buffer(buffer)
                         .set_rotation(webrtc::kVideoRotation_0)
                         .set_timestamp_us(timestamp_us)
                         .build());
}

@end

namespace {

AVCaptureDeviceFormat* SelectClosestFormat(AVCaptureDevice* device,
                                           size_t width,
                                           size_t height) {
  NSArray<AVCaptureDeviceFormat*>* formats =
      [RTCCameraVideoCapturer supportedFormatsForDevice:device];
  AVCaptureDeviceFormat* selectedFormat = nil;
  int currentDiff = INT_MAX;
  for (AVCaptureDeviceFormat* format in formats) {
    CMVideoDimensions dimension =
        CMVideoFormatDescriptionGetDimensions(format.formatDescription);
    int diff = std::abs((int64_t)width - dimension.width) +
               std::abs((int64_t)height - dimension.height);
    if (diff < currentDiff) {
      selectedFormat = format;
      currentDiff = diff;
    }
  }
  return selectedFormat;
}

static NSArray<AVCaptureDevice*>* captureDevices() {
  if (@available(macOS 14, *)) {
    // macOS 14 以上では、新しい API を使用して外部カメラも取得する
    AVCaptureDeviceDiscoverySession* session = [AVCaptureDeviceDiscoverySession
        discoverySessionWithDeviceTypes:@[
          AVCaptureDeviceTypeBuiltInWideAngleCamera, AVCaptureDeviceTypeExternal
        ]
                              mediaType:AVMediaTypeVideo
                               position:AVCaptureDevicePositionUnspecified];
    return session.devices;
  } else {
    // macOS 13 以下では、AVCaptureDeviceDiscoverySessionを使用して内蔵カメラのみ取得する
    AVCaptureDeviceDiscoverySession* session = [AVCaptureDeviceDiscoverySession
        discoverySessionWithDeviceTypes:@[
          AVCaptureDeviceTypeBuiltInWideAngleCamera
        ]
                              mediaType:AVMediaTypeVideo
                               position:AVCaptureDevicePositionUnspecified];
    return session.devices;
  }
}

}  // namespace

MacCapturer::MacCapturer(size_t width,
                         size_t height,
                         size_t target_fps,
                         AVCaptureDevice* device)
    : sora::ScalableVideoTrackSource(sora::ScalableVideoTrackSourceConfig()) {
  RTC_LOG(LS_INFO) << "MacCapturer width=" << width << ", height=" << height
                   << ", target_fps=" << target_fps;

  adapter_ = [[RTCVideoSourceAdapter alloc] init];
  adapter_.capturer = this;

  capturer_ = [[RTCCameraVideoCapturer alloc] initWithDelegate:adapter_];
  AVCaptureDeviceFormat* format = SelectClosestFormat(device, width, height);
  [capturer_ startCaptureWithDevice:device format:format fps:target_fps];
}

webrtc::scoped_refptr<MacCapturer> MacCapturer::Create(
    size_t width,
    size_t height,
    size_t target_fps,
    const std::string& specifiedVideoDevice) {
  AVCaptureDevice* device = FindVideoDevice(specifiedVideoDevice);
  if (!device) {
    RTC_LOG(LS_ERROR) << "Failed to create MacCapture";
    return nullptr;
  }
  return webrtc::make_ref_counted<MacCapturer>(width, height, target_fps,
                                               device);
}

void MacCapturer::ListDevices() {
  NSArray<AVCaptureDevice*>* devices = captureDevices();
  [devices enumerateObjectsUsingBlock:^(AVCaptureDevice* device, NSUInteger i,
                                        BOOL* stop) {
    std::cout << "  [" << i << "] " << [device.localizedName UTF8String]
              << std::endl;
  }];
}

AVCaptureDevice* MacCapturer::FindVideoDevice(
    const std::string& specifiedVideoDevice) {
  // Device の決定ロジックは ffmpeg の avfoundation と同じ仕様にする
  // https://www.ffmpeg.org/ffmpeg-devices.html#avfoundation

  size_t capture_device_index = SIZE_T_MAX;
  NSArray<AVCaptureDevice*>* devices = captureDevices();
  [devices enumerateObjectsUsingBlock:^(AVCaptureDevice* device, NSUInteger i,
                                        BOOL* stop) {
    // 便利なのでデバイスの一覧をログに出力しておく
    RTC_LOG(LS_INFO) << "video device found: [" << i
                     << "] device_name=" << [device.localizedName UTF8String];
  }];

  // video-device オプション未指定、空白、"default", "0" の場合はデフォルトデバイスを返す
  if (specifiedVideoDevice.empty() || specifiedVideoDevice == "default" ||
      specifiedVideoDevice == "0") {
    capture_device_index = 0;
  } else {
    NSUInteger selected_index =
        [devices indexOfObjectPassingTest:^BOOL(AVCaptureDevice* device,
                                                NSUInteger i, BOOL* stop) {
          // デバイス番号を優先して検索
          if (specifiedVideoDevice == [@(i).stringValue UTF8String]) {
            return YES;
          }

          // デバイス名は前方一致検索
          std::string device_name = [device.localizedName UTF8String];
          if (device_name.find(specifiedVideoDevice) == 0) {
            return YES;
          }

          return NO;
        }];

    if (selected_index != NSNotFound) {
      capture_device_index = selected_index;
    }
  }

  if (capture_device_index != SIZE_T_MAX) {
    AVCaptureDevice* device = [devices objectAtIndex:capture_device_index];
    RTC_LOG(LS_INFO) << "selected video device: [" << capture_device_index
                     << "] device_name=" << [device.localizedName UTF8String];
    return device;
  }

  RTC_LOG(LS_INFO) << "no matching video device found";
  return nullptr;
}

void MacCapturer::Destroy() {
  [capturer_ stopCapture];
}

MacCapturer::~MacCapturer() {
  Destroy();
}

void MacCapturer::OnFrame(const webrtc::VideoFrame& frame) {
  OnCapturedFrame(frame);
}
