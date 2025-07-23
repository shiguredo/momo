#ifndef NATIVE_BUFFER_H_
#define NATIVE_BUFFER_H_

// WebRTC
#include <api/video/video_frame.h>
#include <common_video/include/video_frame_buffer.h>
#include <common_video/libyuv/include/webrtc_libyuv.h>
#include <rtc_base/memory/aligned_malloc.h>

class NativeBuffer : public webrtc::VideoFrameBuffer {
 public:
  static webrtc::scoped_refptr<NativeBuffer> Create(webrtc::VideoType video_type,
                                                 int width,
                                                 int height);

  void InitializeData();

  Type type() const override;
  int width() const override;
  int height() const override;
  webrtc::scoped_refptr<webrtc::I420BufferInterface> ToI420() override;

  int RawWidth() const;
  int RawHeight() const;
  void SetScaledSize(int scaled_width, int scaled_height);
  void SetLength(size_t size);
  size_t Length() const;
  webrtc::VideoType VideoType() const;
  const uint8_t* Data() const;
  uint8_t* MutableData();

 protected:
  NativeBuffer(webrtc::VideoType video_type, int width, int height);
  ~NativeBuffer() override;

 private:
  const int raw_width_;
  const int raw_height_;
  int scaled_width_;
  int scaled_height_;
  size_t length_;
  const webrtc::VideoType video_type_;
  const std::unique_ptr<uint8_t, webrtc::AlignedFreeDeleter> data_;
};

#endif  // NATIVE_BUFFER_H_
