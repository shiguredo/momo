#include "jetson_buffer.h"

// Linux
#include <sys/ioctl.h>

// WebRTC
#include <api/video/i420_buffer.h>
#include <rtc_base/logging.h>

static const int kBufferAlignment = 64;

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
    int scaled_height) {
  return new rtc::RefCountedObject<JetsonBuffer>(
      pixfmt,
      raw_width,
      raw_height,
      scaled_width,
      scaled_height);
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

uint32_t JetsonBuffer::V4L2PixelFormat() const {
  return pixfmt_;
}

int JetsonBuffer::DecodedFd() const {
  return fd_;
}

std::shared_ptr<NvJPEGDecoder> JetsonBuffer::JpegDecoder() const {
  return decoder_;
}

uint8_t* JetsonBuffer::Data() const {
  return data_.get();
}

void JetsonBuffer::SetLength(size_t length) {
  length_ = length;
}

size_t JetsonBuffer::Length() const {
  return length_;
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
      decoder_(std::move(decoder)),
      data_(nullptr) {
}

JetsonBuffer::JetsonBuffer(
    uint32_t pixfmt,
    int raw_width,
    int raw_height,
    int scaled_width,
    int scaled_height)
    : pixfmt_(pixfmt),
      raw_width_(raw_width),
      raw_height_(raw_height),
      scaled_width_(scaled_width),
      scaled_height_(scaled_height),
      fd_(-1),
      decoder_(nullptr),
      data_(static_cast<uint8_t*>(
          webrtc::AlignedMalloc(raw_width_ * raw_height * 2,
                                kBufferAlignment))) {
}
