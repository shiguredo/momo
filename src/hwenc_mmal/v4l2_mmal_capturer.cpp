#include "v4l2_mmal_capturer.h"

#include <linux/videodev2.h>
#include <sys/ioctl.h>

#include "modules/video_capture/video_capture_factory.h"
#include "rtc_base/logging.h"
#include "rtc_base/ref_counted_object.h"

#include "rtc/native_buffer.h"

rtc::scoped_refptr<V4L2VideoCapture> V4L2MMALCapture::Create(
    ConnectionSettings cs) {
  rtc::scoped_refptr<V4L2VideoCapture> capturer;
  std::unique_ptr<webrtc::VideoCaptureModule::DeviceInfo> device_info(
      webrtc::VideoCaptureFactory::CreateDeviceInfo());
  if (!device_info) {
    RTC_LOG(LS_ERROR) << "Failed to CreateDeviceInfo";
    return nullptr;
  }

  LogDeviceList(device_info.get());

  for (int i = 0; i < device_info->NumberOfDevices(); ++i) {
    capturer = Create(device_info.get(), cs, i);
    if (capturer) {
      RTC_LOG(LS_INFO) << "Get Capture";
      return capturer;
    }
  }
  RTC_LOG(LS_ERROR) << "Failed to create V4L2VideoCapture";
  return nullptr;
}

rtc::scoped_refptr<V4L2VideoCapture> V4L2MMALCapture::Create(
    webrtc::VideoCaptureModule::DeviceInfo* device_info,
    ConnectionSettings cs,
    size_t capture_device_index) {
  char device_name[256];
  char unique_name[256];
  if (device_info->GetDeviceName(static_cast<uint32_t>(capture_device_index),
                                 device_name, sizeof(device_name), unique_name,
                                 sizeof(unique_name)) != 0) {
    RTC_LOG(LS_WARNING) << "Failed to GetDeviceName";
    return nullptr;
  }
  rtc::scoped_refptr<V4L2VideoCapture> v4l2_capturer(
      new rtc::RefCountedObject<V4L2MMALCapture>());
  if (v4l2_capturer->Init((const char*)&unique_name, cs.video_device) < 0) {
    RTC_LOG(LS_WARNING) << "Failed to create V4L2MMALCapture(" << unique_name
                        << ")";
    return nullptr;
  }
  if (v4l2_capturer->StartCapture(cs) < 0) {
    auto size = cs.getSize();
    RTC_LOG(LS_WARNING) << "Failed to start V4L2MMALCapture(w = " << size.width
                        << ", h = " << size.height << ", fps = " << cs.framerate
                        << ")";
    return nullptr;
  }
  return v4l2_capturer;
}

int32_t V4L2MMALCapture::StartCapture(ConnectionSettings cs) {
  return V4L2VideoCapture::StartCapture(cs);
}

int32_t V4L2MMALCapture::StopCapture() {
  return V4L2VideoCapture::StopCapture();
}

bool V4L2MMALCapture::useNativeBuffer() {
  return true;
}

void V4L2MMALCapture::OnCaptured(struct v4l2_buffer& buf) {
  rtc::scoped_refptr<NativeBuffer> native_buffer(
      NativeBuffer::Create(_captureVideoType, _currentWidth, _currentHeight));
  native_buffer->SetData((unsigned char*)_pool[buf.index].start);
  native_buffer->SetLength(buf.bytesused);
  webrtc::VideoFrame video_frame = webrtc::VideoFrame::Builder()
                                       .set_video_frame_buffer(native_buffer)
                                       .set_timestamp_rtp(0)
                                       .set_timestamp_ms(rtc::TimeMillis())
                                       .set_timestamp_us(rtc::TimeMicros())
                                       .set_rotation(webrtc::kVideoRotation_0)
                                       .build();
  OnCapturedFrame(video_frame);

  // enqueue the buffer again
  if (ioctl(_deviceFd, VIDIOC_QBUF, &buf) == -1) {
    RTC_LOG(LS_INFO) << "Failed to enqueue capture buffer";
  }
}