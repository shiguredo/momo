#include "jetson_buffer.h"

// Linux
#include <sys/ioctl.h>

// WebRTC
#include <api/video/i420_buffer.h>
#include <rtc_base/logging.h>

rtc::scoped_refptr<JetsonBuffer> JetsonBuffer::Create(
    uint32_t pixfmt,
    int raw_width,
    int raw_height,
    int scaled_width,
    int scaled_height,
    int fd,
    std::unique_ptr<NvJPEGDecoder> decoder) {
  return new rtc::RefCountedObject<JetsonBuffer>(
      pixfmt,
      raw_width,
      raw_height,
      scaled_width,
      scaled_height,
      fd,
      std::move(decoder));
}

rtc::scoped_refptr<JetsonBuffer> JetsonBuffer::Create(
    uint32_t pixfmt,
    int raw_width,
    int raw_height,
    int scaled_width,
    int scaled_height,
    int device_fd,
    struct v4l2_buffer* v4l2_buf) {
  return new rtc::RefCountedObject<JetsonBuffer>(
      pixfmt,
      raw_width,
      raw_height,
      scaled_width,
      scaled_height,
      device_fd,
      v4l2_buf);
}

webrtc::VideoFrameBuffer::Type JetsonBuffer::type() const {
  return Type::kNative;
}

int JetsonBuffer::width() const {
  return scaled_width_;
}

int JetsonBuffer::height() const {
  return scaled_height_;
}

rtc::scoped_refptr<webrtc::I420BufferInterface> JetsonBuffer::ToI420() {
  rtc::scoped_refptr<webrtc::I420Buffer> i420_buffer =
      webrtc::I420Buffer::Create(scaled_width_, scaled_height_);
  return i420_buffer;
}

int JetsonBuffer::RawWidth() const {
  return raw_width_;
}

int JetsonBuffer::RawHeight() const {
  return raw_height_;
}

uint32_t JetsonBuffer::PixelFormat() const {
  return pixfmt_;
}

int JetsonBuffer::GetFd() const {
  return fd_;
}

v4l2_buffer* JetsonBuffer::GetV4L2Buffer() const {
  return v4l2_buf_;
}


std::shared_ptr<NvJPEGDecoder> JetsonBuffer::GetDecoder() const {
  return decoder_;
}

JetsonBuffer::JetsonBuffer(
    uint32_t pixfmt,
    int raw_width,
    int raw_height,
    int scaled_width,
    int scaled_height,
    int fd,
    std::unique_ptr<NvJPEGDecoder> decoder)
    : pixfmt_(pixfmt),
      raw_width_(raw_width),
      raw_height_(raw_height),
      scaled_width_(scaled_width),
      scaled_height_(scaled_height),
      fd_(fd),
      device_fd_(-1),
      decoder_(std::move(decoder)) {
}

JetsonBuffer::JetsonBuffer(
    uint32_t pixfmt,
    int raw_width,
    int raw_height,
    int scaled_width,
    int scaled_height,
    int device_fd,
    struct v4l2_buffer* v4l2_buf)
    : pixfmt_(pixfmt),
      raw_width_(raw_width),
      raw_height_(raw_height),
      scaled_width_(scaled_width),
      scaled_height_(scaled_height),
      fd_(-1),
      device_fd_(device_fd),
      v4l2_buf_(v4l2_buf) {
}

JetsonBuffer::~JetsonBuffer() {
  if (device_fd_ != -1) {
    if (ioctl(device_fd_, VIDIOC_QBUF, v4l2_buf_) == -1) {
      RTC_LOG(LS_INFO) << "Failed to enqueue capture buffer";
    }
  }
}
