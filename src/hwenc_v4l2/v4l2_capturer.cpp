#include "v4l2_capturer.h"

// Linux
#include <sys/ioctl.h>

// WebRTC
#include <api/video/nv12_buffer.h>
#include <modules/video_capture/video_capture_factory.h>
#include <rtc_base/logging.h>

#include "v4l2_native_buffer.h"

rtc::scoped_refptr<sora::V4L2VideoCapturer> V4L2Capturer::Create(
    sora::V4L2VideoCapturerConfig config) {
  rtc::scoped_refptr<V4L2VideoCapturer> capturer;
  std::unique_ptr<webrtc::VideoCaptureModule::DeviceInfo> device_info(
      webrtc::VideoCaptureFactory::CreateDeviceInfo());
  if (!device_info) {
    RTC_LOG(LS_ERROR) << "Failed to CreateDeviceInfo";
    return nullptr;
  }

  LogDeviceList(device_info.get());

  for (int i = 0; i < device_info->NumberOfDevices(); ++i) {
    capturer = Create(device_info.get(), config, i);
    if (capturer) {
      RTC_LOG(LS_INFO) << "Get Capture";
      return capturer;
    }
  }
  RTC_LOG(LS_ERROR) << "Failed to create V4L2Capturer";
  return nullptr;
}

V4L2Capturer::V4L2Capturer(const sora::V4L2VideoCapturerConfig& config)
    : sora::V4L2VideoCapturer(config) {}

rtc::scoped_refptr<sora::V4L2VideoCapturer> V4L2Capturer::Create(
    webrtc::VideoCaptureModule::DeviceInfo* device_info,
    sora::V4L2VideoCapturerConfig config,
    size_t capture_device_index) {
  char device_name[256];
  char unique_name[256];

  if (device_info->GetDeviceName(static_cast<uint32_t>(capture_device_index),
                                 device_name, sizeof(device_name), unique_name,
                                 sizeof(unique_name)) != 0) {
    RTC_LOG(LS_WARNING) << "Failed to GetDeviceName";
    return nullptr;
  }

  rtc::scoped_refptr<V4L2Capturer> v4l2_capturer =
      rtc::make_ref_counted<V4L2Capturer>(config);

  if (v4l2_capturer->Init((const char*)&unique_name) < 0) {
    RTC_LOG(LS_WARNING) << "Failed to create V4L2Capturer(" << unique_name
                        << ")";
    return nullptr;
  }

  if (v4l2_capturer->StartCapture(config) < 0) {
    RTC_LOG(LS_WARNING) << "Failed to start V4L2Capturer(w = " << config.width
                        << ", h = " << config.height
                        << ", fps = " << config.framerate << ")";
    return nullptr;
  }

  return v4l2_capturer;
}

void V4L2Capturer::OnCaptured(uint8_t* data, uint32_t bytesused) {
  const int64_t timestamp_us = rtc::TimeMicros();
  int adapted_width, adapted_height, crop_width, crop_height, crop_x, crop_y;
  if (!AdaptFrame(_currentWidth, _currentHeight, timestamp_us, &adapted_width,
                  &adapted_height, &crop_width, &crop_height, &crop_x,
                  &crop_y)) {
    return;
  }

  auto frame_buffer = rtc::make_ref_counted<V4L2NativeBuffer>(
      webrtc::VideoType::kMJPEG, _currentWidth, _currentHeight, adapted_width,
      adapted_height, 0, data, bytesused, _currentWidth, nullptr);

  webrtc::VideoFrame video_frame = webrtc::VideoFrame::Builder()
                                       .set_video_frame_buffer(frame_buffer)
                                       .set_timestamp_rtp(0)
                                       .set_timestamp_ms(rtc::TimeMillis())
                                       .set_timestamp_us(rtc::TimeMicros())
                                       .set_rotation(webrtc::kVideoRotation_0)
                                       .build();
  OnFrame(video_frame);
}