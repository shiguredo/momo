#ifndef MMAL_BUFFER_H_
#define MMAL_BUFFER_H_

extern "C" {
#include <interface/mmal/mmal.h>
}

#include <api/video/video_frame.h>
#include <common_video/include/video_frame_buffer.h>
#include <common_video/libyuv/include/webrtc_libyuv.h>

class MMALBuffer : public webrtc::I420BufferInterface {
 public:
  static rtc::scoped_refptr<MMALBuffer> Create(MMAL_BUFFER_HEADER_T* buffer,
                                               int width,
                                               int height);
  Type type() const override;
  int width() const override;
  int height() const override;
  int StrideY() const override;
  int StrideU() const override;
  int StrideV() const override;
  const uint8_t* DataY() const override;
  const uint8_t* DataU() const override;
  const uint8_t* DataV() const override;

  const size_t length() const;

 protected:
  MMALBuffer(MMAL_BUFFER_HEADER_T* buffer, int width, int height);
  ~MMALBuffer() override;

 private:
  const MMAL_BUFFER_HEADER_T* buffer_;
  const int width_;
  const int height_;
};
#endif  // MMAL_BUFFER_H_
