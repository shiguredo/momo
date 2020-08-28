#include "jetson_v4l2_capturer.h"

// Linux
#include <sys/ioctl.h>

// WebRTC
#include <modules/video_capture/video_capture_factory.h>
#include <rtc_base/logging.h>

// Jetson Linux Multimedia API
#include <NvJpegDecoder.h>

#include "jetson_buffer.h"

#define MJPEG_EOS_SEARCH_SIZE 4096

rtc::scoped_refptr<V4L2VideoCapturer> JetsonV4L2Capturer::Create(
    ConnectionSettings cs) {
  rtc::scoped_refptr<V4L2VideoCapturer> capturer;
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
  RTC_LOG(LS_ERROR) << "Failed to create JetsonV4L2Capturer";
  return nullptr;
}

bool JetsonV4L2Capturer::UseNativeBuffer() {
  return true;  
}

rtc::scoped_refptr<V4L2VideoCapturer> JetsonV4L2Capturer::Create(
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
  rtc::scoped_refptr<V4L2VideoCapturer> v4l2_capturer(
      new rtc::RefCountedObject<JetsonV4L2Capturer>());
  if (v4l2_capturer->Init((const char*)&unique_name, cs.video_device) < 0) {
    RTC_LOG(LS_WARNING) << "Failed to create JetsonV4L2Capturer(" << unique_name
                        << ")";
    return nullptr;
  }
  if (v4l2_capturer->StartCapture(cs) < 0) {
    auto size = cs.GetSize();
    RTC_LOG(LS_WARNING) << "Failed to start JetsonV4L2Capturer(w = "
                        << size.width << ", h = " << size.height
                        << ", fps = " << cs.framerate << ")";
    return nullptr;
  }
  return v4l2_capturer;
}

bool JetsonV4L2Capturer::OnCaptured(struct v4l2_buffer& buf) {
  const int64_t timestamp_us = rtc::TimeMicros();
  int adapted_width, adapted_height, crop_width, crop_height, crop_x, crop_y;
  if (!AdaptFrame(_currentWidth, _currentHeight, timestamp_us, &adapted_width,
                  &adapted_height, &crop_width, &crop_height, &crop_x,
                  &crop_y)) {
    return false;
  }

  if (_captureVideoType == webrtc::VideoType::kMJPEG) {
    unsigned int bytesused = buf.bytesused;
    unsigned int eosSearchSize = MJPEG_EOS_SEARCH_SIZE;
    uint8_t *p;
    /* v4l2_buf.bytesused may have padding bytes for alignment
        Search for EOF to get exact size */
    if (eosSearchSize > bytesused)
        eosSearchSize = bytesused;
    for (unsigned int i = 0; i < eosSearchSize; i++) {
        p = (uint8_t *)_pool[buf.index].start + bytesused;
        if ((*(p-2) == 0xff) && (*(p-1) == 0xd9)) {
            break;
        }
        bytesused--;
    }

    std::unique_ptr<NvJPEGDecoder> decoder(NvJPEGDecoder::createJPEGDecoder("jpegdec"));
    int fd = 0;
    uint32_t width, height, pixfmt;
    if (decoder->decodeToFd(fd, (unsigned char *)_pool[buf.index].start,
        bytesused, pixfmt, width, height) < 0) {
      RTC_LOG(LS_ERROR) << "decodeToFd Failed";
      return false;
    }

    rtc::scoped_refptr<JetsonBuffer> jetson_buffer(
        JetsonBuffer::Create(
            _captureVideoType, width, height, adapted_width, adapted_height,
            fd, pixfmt, std::move(decoder)));
    OnFrame(webrtc::VideoFrame::Builder()
                .set_video_frame_buffer(jetson_buffer)
                .set_timestamp_rtp(0)
                .set_timestamp_ms(rtc::TimeMillis())
                .set_timestamp_us(rtc::TimeMicros())
                .set_rotation(webrtc::kVideoRotation_0)
                .build());

  } else {
    rtc::scoped_refptr<JetsonBuffer> jetson_buffer(
        JetsonBuffer::Create(
            _captureVideoType, _currentWidth, _currentHeight,
            adapted_width, adapted_height));
    RTC_LOG(LS_ERROR) << " buf.bytesused=" << buf.bytesused;
    memcpy(jetson_buffer->Data(), (unsigned char*)_pool[buf.index].start,
           buf.bytesused);
    jetson_buffer->SetLength(buf.bytesused);
    OnFrame(webrtc::VideoFrame::Builder()
                .set_video_frame_buffer(jetson_buffer)
                .set_timestamp_rtp(0)
                .set_timestamp_ms(rtc::TimeMillis())
                .set_timestamp_us(rtc::TimeMicros())
                .set_rotation(webrtc::kVideoRotation_0)
                .build());
  }

  // enqueue the buffer again
  if (ioctl(_deviceFd, VIDIOC_QBUF, &buf) == -1) {
    RTC_LOG(LS_INFO) << "Failed to enqueue capture buffer";
  }
  return true;
}
