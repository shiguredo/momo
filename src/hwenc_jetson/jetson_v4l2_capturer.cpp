#include "jetson_v4l2_capturer.h"

// Linux
#include <sys/ioctl.h>

// WebRTC
#include <modules/video_capture/video_capture_factory.h>
#include <rtc_base/logging.h>

#include "jetson_buffer.h"

rtc::scoped_refptr<V4L2VideoCapturer> JetsonV4L2Capturer::Create(
    V4L2VideoCapturerConfig config) {
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
  RTC_LOG(LS_ERROR) << "Failed to create JetsonV4L2Capturer";
  return nullptr;
}

bool JetsonV4L2Capturer::UseNativeBuffer() {
  return true;
}

rtc::scoped_refptr<V4L2VideoCapturer> JetsonV4L2Capturer::Create(
    webrtc::VideoCaptureModule::DeviceInfo* device_info,
    V4L2VideoCapturerConfig config,
    size_t capture_device_index) {
  char device_name[256];
  char unique_name[256];

  if (device_info->GetDeviceName(static_cast<uint32_t>(capture_device_index),
                                 device_name, sizeof(device_name), unique_name,
                                 sizeof(unique_name)) != 0) {
    RTC_LOG(LS_WARNING) << "Failed to GetDeviceName";
    return nullptr;
  }

  rtc::scoped_refptr<V4L2VideoCapturer> v4l2_capturer(
      new rtc::RefCountedObject<JetsonV4L2Capturer>());

  if (v4l2_capturer->Init((const char*)&unique_name, config.video_device) < 0) {
    RTC_LOG(LS_WARNING) << "Failed to create JetsonV4L2Capturer(" << unique_name
                        << ")";
    return nullptr;
  }

  if (v4l2_capturer->StartCapture(config) < 0) {
    RTC_LOG(LS_WARNING) << "Failed to start JetsonV4L2Capturer(w = "
                        << config.width << ", h = " << config.height
                        << ", fps = " << config.framerate << ")";
    return nullptr;
  }

  return v4l2_capturer;
}

bool JetsonV4L2Capturer::AllocateVideoBuffers() {
  bool result = V4L2VideoCapturer::AllocateVideoBuffers();
  if (result && _captureVideoType == webrtc::VideoType::kMJPEG) {
    std::shared_ptr<JetsonJpegDecoderPool> jpeg_decoder_pool(
        new JetsonJpegDecoderPool());
    jpeg_decoder_pool_ = jpeg_decoder_pool;
  }
  return result;
}

bool JetsonV4L2Capturer::DeAllocateVideoBuffers() {
  jpeg_decoder_pool_ = nullptr;
  return V4L2VideoCapturer::DeAllocateVideoBuffers();
}

void JetsonV4L2Capturer::OnCaptured(uint8_t* data, uint32_t bytesused) {
  const int64_t timestamp_us = rtc::TimeMicros();
  int adapted_width, adapted_height, crop_width, crop_height, crop_x, crop_y;
  if (!AdaptFrame(_currentWidth, _currentHeight, timestamp_us, &adapted_width,
                  &adapted_height, &crop_width, &crop_height, &crop_x,
                  &crop_y)) {
    return;
  }

  if (_captureVideoType == webrtc::VideoType::kMJPEG) {
    auto decoder = jpeg_decoder_pool_->Pop();
    int fd = 0;
    uint32_t width, height, pixfmt;
    if (decoder->DecodeToFd(fd, data, bytesused, pixfmt, width, height) < 0) {
      RTC_LOG(LS_ERROR) << "decodeToFd Failed";
      return;
    }

    rtc::scoped_refptr<JetsonBuffer> jetson_buffer(
        JetsonBuffer::Create(_captureVideoType, width, height, adapted_width,
                             adapted_height, fd, pixfmt, std::move(decoder)));
    OnFrame(webrtc::VideoFrame::Builder()
                .set_video_frame_buffer(jetson_buffer)
                .set_timestamp_rtp(0)
                .set_timestamp_ms(rtc::TimeMillis())
                .set_timestamp_us(rtc::TimeMicros())
                .set_rotation(webrtc::kVideoRotation_0)
                .build());

  } else {
    rtc::scoped_refptr<JetsonBuffer> jetson_buffer(
        JetsonBuffer::Create(_captureVideoType, _currentWidth, _currentHeight,
                             adapted_width, adapted_height));
    memcpy(jetson_buffer->Data(), data, bytesused);
    jetson_buffer->SetLength(bytesused);
    OnFrame(webrtc::VideoFrame::Builder()
                .set_video_frame_buffer(jetson_buffer)
                .set_timestamp_rtp(0)
                .set_timestamp_ms(rtc::TimeMillis())
                .set_timestamp_us(rtc::TimeMicros())
                .set_rotation(webrtc::kVideoRotation_0)
                .build());
  }
}