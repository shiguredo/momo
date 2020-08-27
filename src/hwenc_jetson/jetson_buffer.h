#ifndef JETSON_BUFFER_H_
#define JETSON_BUFFER_H_

#include <memory>
#include <linux/videodev2.h>

// WebRTC
#include <api/video/video_frame.h>
#include <common_video/include/video_frame_buffer.h>

// Jetson Linux Multimedia API
#include "nvbuf_utils.h"
#include "NvJpegDecoder.h"

class JetsonBuffer : public webrtc::VideoFrameBuffer {
 public:
  static rtc::scoped_refptr<JetsonBuffer> Create(
    uint32_t pixfmt,
    int raw_width,
    int raw_height,
    int scaled_width,
    int scaled_height,
    int fd,
    std::unique_ptr<NvJPEGDecoder> decoder);

  static rtc::scoped_refptr<JetsonBuffer> Create(
    uint32_t pixfmt,
    int raw_width,
    int raw_height,
    int scaled_width,
    int scaled_height,
    int fd,
    int device_fd,
    struct v4l2_buffer* v4l2_buf);

  Type type() const override;
  int width() const override;
  int height() const override;
  rtc::scoped_refptr<webrtc::I420BufferInterface> ToI420() override;

  int RawWidth() const;
  int RawHeight() const;
  uint32_t PixelFormat() const;
  int GetFd() const;
  std::shared_ptr<NvJPEGDecoder> GetDecoder() const;

 protected:
  JetsonBuffer(
    uint32_t pixfmt,
    int raw_width,
    int raw_height,
    int scaled_width,
    int scaled_height,
    int fd,
    std::unique_ptr<NvJPEGDecoder> decoder);

  JetsonBuffer(
    uint32_t pixfmt,
    int raw_width,
    int raw_height,
    int scaled_width,
    int scaled_height,
    int fd,
    int device_fd,
    struct v4l2_buffer* v4l2_buf);
  ~JetsonBuffer() override;

 private:
  uint32_t pixfmt_;
  const int raw_width_;
  const int raw_height_;
  const int scaled_width_;
  const int scaled_height_;
  const int fd_;
  const int device_fd_;
  struct v4l2_buffer* v4l2_buf_;
  std::shared_ptr<NvJPEGDecoder> decoder_;
};
#endif  // JETSON_BUFFER_H_
