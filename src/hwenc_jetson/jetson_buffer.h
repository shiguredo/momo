#ifndef JETSON_BUFFER_H_
#define JETSON_BUFFER_H_

#include <memory>
#include <linux/videodev2.h>

// WebRTC
#include <api/video/video_frame.h>
#include <common_video/include/video_frame_buffer.h>
#include <rtc_base/memory/aligned_malloc.h>

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
    int scaled_height);

  Type type() const override;
  int width() const override;
  int height() const override;
  rtc::scoped_refptr<webrtc::I420BufferInterface> ToI420() override;

  int RawWidth() const;
  int RawHeight() const;
  uint32_t V4L2PixelFormat() const;
  int DecodedFd() const;
  std::shared_ptr<NvJPEGDecoder> JpegDecoder() const;
  uint8_t* Data() const;
  void SetLength(size_t size);
  size_t Length() const;

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
    int scaled_height);

 private:
  uint32_t pixfmt_;
  const int raw_width_;
  const int raw_height_;
  const int scaled_width_;
  const int scaled_height_;
  const int fd_;
  const std::shared_ptr<NvJPEGDecoder> decoder_;
  const std::unique_ptr<uint8_t, webrtc::AlignedFreeDeleter> data_;
  size_t length_;
};
#endif  // JETSON_BUFFER_H_
