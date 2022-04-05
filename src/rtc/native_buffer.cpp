#include "native_buffer.h"

// WebRTC
#include <api/video/i420_buffer.h>
#include <rtc_base/checks.h>
#include <third_party/libyuv/include/libyuv.h>

static const int kBufferAlignment = 64;

namespace {

int ArgbDataSize(int height, int width) {
  return width * height * 4;
}

}  // namespace

rtc::scoped_refptr<NativeBuffer>
NativeBuffer::Create(webrtc::VideoType video_type, int width, int height) {
  return rtc::make_ref_counted<NativeBuffer>(video_type, width, height);
}

webrtc::VideoFrameBuffer::Type NativeBuffer::type() const {
  return Type::kNative;
}

void NativeBuffer::InitializeData() {
  memset(data_.get(), 0, ArgbDataSize(raw_height_, raw_width_));
}

int NativeBuffer::width() const {
  return scaled_width_;
}

int NativeBuffer::height() const {
  return scaled_height_;
}

rtc::scoped_refptr<webrtc::I420BufferInterface> NativeBuffer::ToI420() {
  rtc::scoped_refptr<webrtc::I420Buffer> i420_buffer =
      webrtc::I420Buffer::Create(raw_width_, raw_height_);
  const int conversionResult = libyuv::ConvertToI420(
      data_.get(), length_, i420_buffer.get()->MutableDataY(),
      i420_buffer.get()->StrideY(), i420_buffer.get()->MutableDataU(),
      i420_buffer.get()->StrideU(), i420_buffer.get()->MutableDataV(),
      i420_buffer.get()->StrideV(), 0, 0, raw_width_, raw_height_, raw_width_,
      raw_height_, libyuv::kRotate0, ConvertVideoType(video_type_));
  rtc::scoped_refptr<webrtc::I420Buffer> scaled_buffer =
      webrtc::I420Buffer::Create(scaled_width_, scaled_height_);
  scaled_buffer->ScaleFrom(*i420_buffer->ToI420());
  return scaled_buffer;
}

int NativeBuffer::RawWidth() const {
  return raw_width_;
}

int NativeBuffer::RawHeight() const {
  return raw_height_;
}

void NativeBuffer::SetScaledSize(int scaled_width, int scaled_height) {
  scaled_width_ = scaled_width;
  scaled_height_ = scaled_height;
}

void NativeBuffer::SetLength(size_t length) {
  length_ = length;
}

size_t NativeBuffer::Length() const {
  return length_;
}

webrtc::VideoType NativeBuffer::VideoType() const {
  return video_type_;
}

const uint8_t* NativeBuffer::Data() const {
  return data_.get();
}

uint8_t* NativeBuffer::MutableData() {
  return const_cast<uint8_t*>(Data());
}

NativeBuffer::NativeBuffer(webrtc::VideoType video_type, int width, int height)
    : raw_width_(width),
      raw_height_(height),
      scaled_width_(width),
      scaled_height_(height),
      length_(ArgbDataSize(height, width)),
      video_type_(video_type),
      data_(static_cast<uint8_t*>(
          webrtc::AlignedMalloc(ArgbDataSize(height, width),
                                kBufferAlignment))) {}

NativeBuffer::~NativeBuffer() {}
