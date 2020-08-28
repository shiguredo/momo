#include "jetson_buffer.h"

// Linux
#include <sys/ioctl.h>

// WebRTC
#include <api/video/i420_buffer.h>
#include <rtc_base/logging.h>
#include <third_party/libyuv/include/libyuv.h>

static const int kBufferAlignment = 64;

rtc::scoped_refptr<JetsonBuffer> JetsonBuffer::Create(
    webrtc::VideoType video_type,
    int raw_width,
    int raw_height,
    int scaled_width,
    int scaled_height,
    int fd,
    uint32_t pixfmt,
    std::unique_ptr<NvJPEGDecoder> decoder) {
  return new rtc::RefCountedObject<JetsonBuffer>(
      video_type,
      raw_width,
      raw_height,
      scaled_width,
      scaled_height,
      fd,
      pixfmt,
      std::move(decoder));
}

rtc::scoped_refptr<JetsonBuffer> JetsonBuffer::Create(
    webrtc::VideoType video_type,
    int raw_width,
    int raw_height,
    int scaled_width,
    int scaled_height) {
  return new rtc::RefCountedObject<JetsonBuffer>(
      video_type,
      raw_width,
      raw_height,
      scaled_width,
      scaled_height);
}

webrtc::VideoFrameBuffer::Type JetsonBuffer::type() const {
  return Type::kNative;
}


webrtc::VideoType JetsonBuffer::VideoType() const {
  return video_type_;
}

int JetsonBuffer::width() const {
  return scaled_width_;
}

int JetsonBuffer::height() const {
  return scaled_height_;
}

rtc::scoped_refptr<webrtc::I420BufferInterface> JetsonBuffer::ToI420() {
  rtc::scoped_refptr<webrtc::I420Buffer> i420_buffer =
      webrtc::I420Buffer::Create(raw_width_, raw_height_);
  if (video_type_ == webrtc::VideoType::kMJPEG) {
    NvBufferParams params = {0};
    if (NvBufferGetParams(fd_, &params) == -1) {
      RTC_LOG(LS_ERROR) << __FUNCTION__ << " Failed to NvBufferGetParams";
      return i420_buffer;
    }
    int ret;
    void *data_addr;
    uint8_t* dest_addr;
    for (int plane = 0; plane < params.num_planes; plane++) {
      ret = NvBufferMemMap (fd_, plane, NvBufferMem_Read, &data_addr);
      if (ret == 0) {
        NvBufferMemSyncForCpu (fd_, plane, &data_addr);
        int height;
        int v_stride;
        if (plane == 0) {
          dest_addr = i420_buffer.get()->MutableDataY();
          height = raw_height_;
          v_stride = 1;
        } else if (plane == 1) {
          dest_addr = i420_buffer.get()->MutableDataU();
          height = (raw_height_ + 1) >> 1;
          v_stride = 2;
        } else if (plane == 2) {
          dest_addr = i420_buffer.get()->MutableDataV();
          height = (raw_height_ + 1) >> 1;
          v_stride = 2;
        }
        for (int i = 0; i < height; i++)
        {
          memcpy(dest_addr + params.width[plane] * i, 
                (uint8_t*)data_addr + params.pitch[plane] * i * v_stride,
                params.width[plane]);
        }
      }
      NvBufferMemUnMap (fd_, plane, &data_addr);
      if (ret == -1) {
        RTC_LOG(LS_ERROR) << __FUNCTION__ << " Failed to NvBufferMemMap plane=" << plane;
        return i420_buffer;
      }
    }
  } else {
    const int conversionResult = libyuv::ConvertToI420(
        data_.get(), length_, i420_buffer.get()->MutableDataY(),
        i420_buffer.get()->StrideY(), i420_buffer.get()->MutableDataU(),
        i420_buffer.get()->StrideU(), i420_buffer.get()->MutableDataV(),
        i420_buffer.get()->StrideV(), 0, 0, raw_width_, raw_height_, raw_width_,
        raw_height_, libyuv::kRotate0, ConvertVideoType(video_type_));
  }
  if (raw_width_ == scaled_width_ && raw_height_ == scaled_height_) {
    return i420_buffer;
  }
  rtc::scoped_refptr<webrtc::I420Buffer> scaled_buffer =
      webrtc::I420Buffer::Create(scaled_width_, scaled_height_);
  scaled_buffer->ScaleFrom(*i420_buffer->ToI420());
  return scaled_buffer;
}

int JetsonBuffer::RawWidth() const {
  return raw_width_;
}

int JetsonBuffer::RawHeight() const {
  return raw_height_;
}

int JetsonBuffer::DecodedFd() const {
  return fd_;
}

uint32_t JetsonBuffer::V4L2PixelFormat() const {
  return pixfmt_;
}

std::shared_ptr<NvJPEGDecoder> JetsonBuffer::JpegDecoder() const {
  return decoder_;
}

uint8_t* JetsonBuffer::Data() const {
  return data_.get();
}

void JetsonBuffer::SetLength(size_t length) {
  length_ = length;
}

size_t JetsonBuffer::Length() const {
  return length_;
}

JetsonBuffer::JetsonBuffer(
    webrtc::VideoType video_type,
    int raw_width,
    int raw_height,
    int scaled_width,
    int scaled_height,
    int fd,
    uint32_t pixfmt,
    std::unique_ptr<NvJPEGDecoder> decoder)
    : video_type_(video_type),
      raw_width_(raw_width),
      raw_height_(raw_height),
      scaled_width_(scaled_width),
      scaled_height_(scaled_height),
      fd_(fd),
      pixfmt_(pixfmt),
      decoder_(std::move(decoder)),
      data_(nullptr) {
}

JetsonBuffer::JetsonBuffer(
    webrtc::VideoType video_type,
    int raw_width,
    int raw_height,
    int scaled_width,
    int scaled_height)
    : video_type_(video_type),
      raw_width_(raw_width),
      raw_height_(raw_height),
      scaled_width_(scaled_width),
      scaled_height_(scaled_height),
      fd_(-1),
      pixfmt_(0),
      decoder_(nullptr),
      data_(static_cast<uint8_t*>(
          webrtc::AlignedMalloc(
              webrtc::CalcBufferSize(video_type, raw_width, raw_height),
              kBufferAlignment))) {
}
