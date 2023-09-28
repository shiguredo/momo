#include "v4l2_native_buffer.h"

// WebRTC
#include <rtc_base/logging.h>

V4L2NativeBuffer::V4L2NativeBuffer(webrtc::VideoType video_type,
                                   int raw_width,
                                   int raw_height,
                                   int scaled_width,
                                   int scaled_height,
                                   int fd,
                                   const uint8_t* data,
                                   int size,
                                   int stride,
                                   std::function<void()> on_destruction)
    : video_type_(video_type),
      raw_width_(raw_width),
      raw_height_(raw_height),
      scaled_width_(scaled_width),
      scaled_height_(scaled_height),
      fd_(fd),
      size_(size),
      stride_(stride) {
  if (data != nullptr) {
    data_.reset(new uint8_t[size_]);
    memcpy(data_.get(), data, size_);
  }
  if (on_destruction != nullptr) {
    shared_on_destruction_.reset(new int(),
                                 [on_destruction](int*) { on_destruction(); });
  }
}

V4L2NativeBuffer::V4L2NativeBuffer(webrtc::VideoType video_type,
                                   int raw_width,
                                   int raw_height,
                                   int scaled_width,
                                   int scaled_height,
                                   int fd,
                                   const std::shared_ptr<uint8_t> data,
                                   int size,
                                   int stride,
                                   std::shared_ptr<void> shared_on_destruction)
    : video_type_(video_type),
      raw_width_(raw_width),
      raw_height_(raw_height),
      scaled_width_(scaled_width),
      scaled_height_(scaled_height),
      fd_(fd),
      data_(data),
      size_(size),
      stride_(stride),
      shared_on_destruction_(shared_on_destruction) {}

webrtc::VideoFrameBuffer::Type V4L2NativeBuffer::type() const {
  return webrtc::VideoFrameBuffer::Type::kNative;
};
int V4L2NativeBuffer::width() const {
  return scaled_width_;
}
int V4L2NativeBuffer::height() const {
  return scaled_height_;
}
rtc::scoped_refptr<webrtc::I420BufferInterface> V4L2NativeBuffer::ToI420() {
  RTC_LOG(LS_ERROR) << "V4L2NativeBuffer::ToI420() not implemented";
  return nullptr;
}

// crop は無視してサイズだけ変更する
rtc::scoped_refptr<webrtc::VideoFrameBuffer> V4L2NativeBuffer::CropAndScale(
    int offset_x,
    int offset_y,
    int crop_width,
    int crop_height,
    int scaled_width,
    int scaled_height) {
  return rtc::make_ref_counted<V4L2NativeBuffer>(
      video_type_, raw_width_, raw_height_, scaled_width, scaled_height, fd_,
      data_, size_, stride_, shared_on_destruction_);
}

webrtc::VideoType V4L2NativeBuffer::video_type() const {
  return video_type_;
}
int V4L2NativeBuffer::fd() const {
  return fd_;
}
std::shared_ptr<uint8_t> V4L2NativeBuffer::data() const {
  return data_;
}
int V4L2NativeBuffer::size() const {
  return size_;
}
int V4L2NativeBuffer::stride() const {
  return stride_;
}
int V4L2NativeBuffer::raw_width() const {
  return raw_width_;
}
int V4L2NativeBuffer::raw_height() const {
  return raw_height_;
}
