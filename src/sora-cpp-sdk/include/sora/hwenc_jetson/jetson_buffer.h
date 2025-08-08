#ifndef SORA_HWENC_JETSON_JETSON_BUFFER_H_
#define SORA_HWENC_JETSON_JETSON_BUFFER_H_

#include <linux/videodev2.h>
#include <memory>

// WebRTC
#include <api/video/video_frame.h>
#include <common_video/include/video_frame_buffer.h>
#include <common_video/libyuv/include/webrtc_libyuv.h>
#include <rtc_base/memory/aligned_malloc.h>

#include "jetson_jpeg_decoder.h"

namespace sora {

class JetsonBuffer : public webrtc::VideoFrameBuffer {
 public:
  static webrtc::scoped_refptr<JetsonBuffer> Create(
      webrtc::VideoType video_type,
      int raw_width,
      int raw_height,
      int scaled_width,
      int scaled_height,
      int fd,
      uint32_t pixfmt,
      std::shared_ptr<JetsonJpegDecoder> decoder);

  static webrtc::scoped_refptr<JetsonBuffer> Create(
      webrtc::VideoType video_type,
      int raw_width,
      int raw_height,
      int scaled_width,
      int scaled_height);

  Type type() const override;
  webrtc::VideoType VideoType() const;
  int width() const override;
  int height() const override;
  webrtc::scoped_refptr<webrtc::I420BufferInterface> ToI420() override;

  int RawWidth() const;
  int RawHeight() const;
  int DecodedFd() const;
  uint32_t V4L2PixelFormat() const;
  std::shared_ptr<JetsonJpegDecoder> JpegDecoder() const;
  uint8_t* Data() const;
  void SetLength(size_t size);
  size_t Length() const;

 protected:
  JetsonBuffer(webrtc::VideoType video_type,
               int raw_width,
               int raw_height,
               int scaled_width,
               int scaled_height,
               int fd,
               uint32_t pixfmt,
               std::shared_ptr<JetsonJpegDecoder> decoder);

  JetsonBuffer(webrtc::VideoType video_type,
               int raw_width,
               int raw_height,
               int scaled_width,
               int scaled_height);

 private:
  webrtc::VideoType video_type_;
  const int raw_width_;
  const int raw_height_;
  const int scaled_width_;
  const int scaled_height_;
  const int fd_;
  const uint32_t pixfmt_;
  const std::shared_ptr<JetsonJpegDecoder> decoder_;
  const std::unique_ptr<uint8_t, webrtc::AlignedFreeDeleter> data_;
  size_t length_;
};

}  // namespace sora

#endif
