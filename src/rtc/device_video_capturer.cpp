/*
 *  Copyright (c) 2013 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "device_video_capturer.h"

#include <stdint.h>

#include <memory>

// WebRTC
#include <modules/video_capture/video_capture_factory.h>
#include <rtc_base/checks.h>
#include <rtc_base/logging.h>

DeviceVideoCapturer::DeviceVideoCapturer()
    : sora::ScalableVideoTrackSource(sora::ScalableVideoTrackSourceConfig()),
      vcm_(nullptr) {}

DeviceVideoCapturer::~DeviceVideoCapturer() {
  Destroy();
}

bool DeviceVideoCapturer::Init(size_t width,
                               size_t height,
                               size_t target_fps,
                               size_t capture_device_index) {
  std::unique_ptr<webrtc::VideoCaptureModule::DeviceInfo> device_info(
      webrtc::VideoCaptureFactory::CreateDeviceInfo());

  char device_name[256];
  char unique_name[256];
  if (device_info->GetDeviceName(static_cast<uint32_t>(capture_device_index),
                                 device_name, sizeof(device_name), unique_name,
                                 sizeof(unique_name)) != 0) {
    Destroy();
    return false;
  }

  vcm_ = webrtc::VideoCaptureFactory::Create(unique_name);
  if (!vcm_) {
    return false;
  }
  vcm_->RegisterCaptureDataCallback(this);

  device_info->GetCapability(vcm_->CurrentDeviceName(), 0, capability_);

  capability_.width = static_cast<int32_t>(width);
  capability_.height = static_cast<int32_t>(height);
  capability_.maxFPS = static_cast<int32_t>(target_fps);
  capability_.videoType = webrtc::VideoType::kI420;

  if (vcm_->StartCapture(capability_) != 0) {
    Destroy();
    return false;
  }

  RTC_CHECK(vcm_->CaptureStarted());

  return true;
}

rtc::scoped_refptr<DeviceVideoCapturer>
DeviceVideoCapturer::Create(size_t width, size_t height, size_t target_fps) {
  rtc::scoped_refptr<DeviceVideoCapturer> capturer;
  std::unique_ptr<webrtc::VideoCaptureModule::DeviceInfo> info(
      webrtc::VideoCaptureFactory::CreateDeviceInfo());
  if (!info) {
    RTC_LOG(LS_WARNING) << "Failed to CreateDeviceInfo";
    return nullptr;
  }
  int num_devices = info->NumberOfDevices();
  for (int i = 0; i < num_devices; ++i) {
    capturer = Create(width, height, target_fps, i);
    if (capturer) {
      RTC_LOG(LS_WARNING) << "Get Capture";
      return capturer;
    }
  }
  RTC_LOG(LS_WARNING) << "Failed to create DeviceVideoCapturer";

  return nullptr;
}

rtc::scoped_refptr<DeviceVideoCapturer> DeviceVideoCapturer::Create(
    size_t width,
    size_t height,
    size_t target_fps,
    size_t capture_device_index) {
  rtc::scoped_refptr<DeviceVideoCapturer> vcm_capturer(
      new rtc::RefCountedObject<DeviceVideoCapturer>());
  if (!vcm_capturer->Init(width, height, target_fps, capture_device_index)) {
    RTC_LOG(LS_WARNING) << "Failed to create DeviceVideoCapturer(w = " << width
                        << ", h = " << height << ", fps = " << target_fps
                        << ")";
    return nullptr;
  }
  return vcm_capturer;
}

rtc::scoped_refptr<DeviceVideoCapturer> DeviceVideoCapturer::Create(
    size_t width,
    size_t height,
    size_t target_fps,
    const std::string& capture_device) {
  rtc::scoped_refptr<DeviceVideoCapturer> vcm_capturer(
      new rtc::RefCountedObject<DeviceVideoCapturer>());

  // 便利なのでデバイスの一覧をログに出力しておく
  if (vcm_capturer->LogDeviceInfo() != 0) {
    return nullptr;
  }

  // デバイス指定なし
  if (capture_device.empty()) {
    return Create(width, height, target_fps);
  }

  auto index = vcm_capturer->GetDeviceIndex(capture_device);
  if (index < 0) {
    return nullptr;
  }
  return Create(width, height, target_fps, static_cast<size_t>(index));
}

void DeviceVideoCapturer::Destroy() {
  if (!vcm_)
    return;

  vcm_->StopCapture();
  vcm_->DeRegisterCaptureDataCallback();
  // Release reference to VCM.
  vcm_ = nullptr;
}

void DeviceVideoCapturer::OnFrame(const webrtc::VideoFrame& frame) {
  OnCapturedFrame(frame);
}

int DeviceVideoCapturer::LogDeviceInfo() {
  std::unique_ptr<webrtc::VideoCaptureModule::DeviceInfo> info(
      webrtc::VideoCaptureFactory::CreateDeviceInfo());
  if (!info) {
    RTC_LOG(LS_ERROR) << "Failed to CreateDeviceInfo";
    return -1;
  }
  int num_devices = info->NumberOfDevices();
  for (int i = 0; i < num_devices; ++i) {
    const uint32_t kSize = 256;
    char name[kSize] = {0};
    char id[kSize] = {0};
    if (info->GetDeviceName(i, name, kSize, id, kSize) != 0) {
      RTC_LOG(LS_WARNING) << "Failed to GetDeviceName(" << i << ")";
      continue;
    }
    RTC_LOG(LS_INFO) << "GetDeviceName(" << i << "): device_name=" << name
                     << ", unique_name=" << id;
  }
  return 0;
}

int DeviceVideoCapturer::GetDeviceIndex(const std::string& device) {
  std::unique_ptr<webrtc::VideoCaptureModule::DeviceInfo> info(
      webrtc::VideoCaptureFactory::CreateDeviceInfo());
  if (!info) {
    RTC_LOG(LS_ERROR) << "Failed to CreateDeviceInfo";
    return -1;
  }

  int ndev = -1;
  // 指定されたdeviceがすべて数字で構成されているかどうかを確認
  // してから数値に変換
  if (std::all_of(device.cbegin(), device.cend(),
                  [](char ch) { return std::isdigit(ch); })) {
    try {
      ndev = std::stoi(device);
    } catch (const std::exception&) {
      ndev = -1;
    }
  }

  auto size = device.size();
  int num_devices = info->NumberOfDevices();
  for (int i = 0; i < num_devices; ++i) {
    const uint32_t kSize = 256;
    char name[kSize] = {0};
    char mid[kSize] = {0};
    if (info->GetDeviceName(static_cast<uint32_t>(i), name, kSize, mid,
                            kSize) != -1) {
      // デバイスidでの検索
      if (i == ndev) {
        return i;
      }
      // デバイスmidでの前方一致検索
      std::string candidate_mid{mid};
      if (candidate_mid.size() >= size &&
          std::equal(std::begin(device), std::end(device),
                     std::begin(candidate_mid))) {
        return i;
      }
      // デバイス名での前方一致検索
      std::string candidate_name{name};
      if (candidate_name.size() >= size &&
          std::equal(std::begin(device), std::end(device),
                     std::begin(candidate_name))) {
        return i;
      }
    }
  }
  return -1;
}
