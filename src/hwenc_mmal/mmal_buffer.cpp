#include "mmal_buffer.h"

// WebRTC
#include <api/video/i420_buffer.h>
#include <rtc_base/checks.h>
#include <third_party/libyuv/include/libyuv.h>

rtc::scoped_refptr<MMALBuffer> MMALBuffer::Create(MMAL_BUFFER_HEADER_T* buffer,
                                                  int width,
                                                  int height) {
  return new rtc::RefCountedObject<MMALBuffer>(buffer, width, height);
}

webrtc::VideoFrameBuffer::Type MMALBuffer::type() const {
  return Type::kNative;
}

int MMALBuffer::width() const {
  return width_;
}

int MMALBuffer::height() const {
  return height_;
}

int MMALBuffer::StrideY() const {
  return width_;
}

int MMALBuffer::StrideU() const {
  return (width_ + 1) / 2;
}

int MMALBuffer::StrideV() const {
  return (width_ + 1) / 2;
}

const uint8_t* MMALBuffer::DataY() const {
  return buffer_->data;
}

const uint8_t* MMALBuffer::DataU() const {
  return DataY() + VCOS_ALIGN_UP(width_, 32) * VCOS_ALIGN_UP(height_, 16);
}

const uint8_t* MMALBuffer::DataV() const {
  return DataY() + VCOS_ALIGN_UP(width_, 32) * VCOS_ALIGN_UP(height_, 16) +
         StrideU() * ((VCOS_ALIGN_UP(height_, 16) + 1) / 2);
}

const size_t MMALBuffer::length() const {
  return buffer_->length;
}

MMALBuffer::MMALBuffer(MMAL_BUFFER_HEADER_T* buffer, int width, int height)
    : buffer_(buffer), width_(width), height_(height) {}

MMALBuffer::~MMALBuffer() {
  mmal_buffer_header_release((MMAL_BUFFER_HEADER_T*)buffer_);
}
