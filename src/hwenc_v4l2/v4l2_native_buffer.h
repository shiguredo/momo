#ifndef V4L2_NATIVE_BUFFER_H_
#define V4L2_NATIVE_BUFFER_H_

#include <functional>

// WebRTC
#include <api/scoped_refptr.h>
#include <api/video/video_frame_buffer.h>
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
                   const uint8_t* data,
                   int size,
                   int stride,
                   std::function<void()> on_destruction);

  V4L2NativeBuffer(webrtc::VideoType video_type,
                   int raw_width,
                   int raw_height,
                   int scaled_width,
                   int scaled_height,
                   int fd,
                   const std::shared_ptr<uint8_t> data,
                   int size,
                   int stride,
                   std::shared_ptr<void> shared_on_destruction);

  webrtc::VideoFrameBuffer::Type type() const override;
  int width() const override;
  int height() const override;
  rtc::scoped_refptr<webrtc::I420BufferInterface> ToI420() override;

  // crop は無視してサイズだけ変更する
  rtc::scoped_refptr<webrtc::VideoFrameBuffer> CropAndScale(
      int offset_x,
      int offset_y,
      int crop_width,
      int crop_height,
      int scaled_width,
      int scaled_height) override;

  webrtc::VideoType video_type() const;
  int fd() const;
  std::shared_ptr<uint8_t> data() const;
  int size() const;
  int stride() const;
  int raw_width() const;
  int raw_height() const;

 private:
  webrtc::VideoType video_type_;
  int raw_width_;
  int raw_height_;
  int scaled_width_;
  int scaled_height_;
  int fd_;
  std::shared_ptr<uint8_t> data_;
  int size_;
  int stride_;
  std::shared_ptr<void> shared_on_destruction_;
};

#endif
