#ifndef V4L2_NATIVE_BUFFER_H_
#define V4L2_NATIVE_BUFFER_H_

// WebRTC
#include <common_video/include/video_frame_buffer.h>
#include <common_video/libyuv/include/webrtc_libyuv.h>

class V4L2NativeBuffer : public webrtc::VideoFrameBuffer {
 public:
  V4L2NativeBuffer(webrtc::VideoType video_type,
                   int raw_width,
                   int raw_height,
                   int scaled_width,
                   int scaled_height,
                   int fd,
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
        stride_(stride),
        on_destruction_(on_destruction) {}

  ~V4L2NativeBuffer() { on_destruction_(); }

  webrtc::VideoFrameBuffer::Type type() const override {
    return webrtc::VideoFrameBuffer::Type::kNative;
  };
  int width() const override { return scaled_width_; }
  int height() const override { return scaled_height_; }
  rtc::scoped_refptr<webrtc::I420BufferInterface> ToI420() override {
    RTC_LOG(LS_ERROR) << "V4L2NativeBuffer::ToI420() not implemented";
    return nullptr;
  }

  // rtc::scoped_refptr<VideoFrameBuffer> CropAndScale(
  //     int offset_x,
  //     int offset_y,
  //     int crop_width,
  //     int crop_height,
  //     int scaled_width,
  //     int scaled_height) override {
  //   RTC_LOG(LS_INFO) << "V4L2NativeBuffer::CropAndScale() called";
  //   on_destruction_ = nullptr;
  //   return rtc::make_ref_counted<V4L2NativeBuffer>(
  //       video_type_, raw_width_, raw_height_, scaled_width, scaled_height, fd_,
  //       size_, stride_, on_destruction_);
  // }

  int fd() const { return fd_; }
  int size() const { return size_; }
  int stride() const { return stride_; }
  int raw_width() const { return raw_width_; }
  int raw_height() const { return raw_height_; }

 private:
  webrtc::VideoType video_type_;
  int raw_width_;
  int raw_height_;
  int scaled_width_;
  int scaled_height_;
  int fd_;
  int size_;
  int stride_;
  std::function<void()> on_destruction_;
};

#endif
