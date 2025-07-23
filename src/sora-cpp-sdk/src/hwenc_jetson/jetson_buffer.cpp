#include "sora/hwenc_jetson/jetson_buffer.h"

// Linux
#include <sys/ioctl.h>

// WebRTC
#include <api/video/i420_buffer.h>
#include <rtc_base/logging.h>
#include <third_party/libyuv/include/libyuv.h>

// Jetson Linux Multimedia API
#include <nvbufsurface.h>
#include <nvbufsurftransform.h>

namespace sora {

static const int kBufferAlignment = 64;

webrtc::scoped_refptr<JetsonBuffer> JetsonBuffer::Create(
    webrtc::VideoType video_type,
    int raw_width,
    int raw_height,
    int scaled_width,
    int scaled_height,
    int fd,
    uint32_t pixfmt,
    std::shared_ptr<JetsonJpegDecoder> decoder) {
  return webrtc::make_ref_counted<JetsonBuffer>(video_type, raw_width, raw_height,
                                             scaled_width, scaled_height, fd,
                                             pixfmt, decoder);
}

webrtc::scoped_refptr<JetsonBuffer> JetsonBuffer::Create(
    webrtc::VideoType video_type,
    int raw_width,
    int raw_height,
    int scaled_width,
    int scaled_height) {
  return webrtc::make_ref_counted<JetsonBuffer>(video_type, raw_width, raw_height,
                                             scaled_width, scaled_height);
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

webrtc::scoped_refptr<webrtc::I420BufferInterface> JetsonBuffer::ToI420() {
  if (video_type_ == webrtc::VideoType::kMJPEG) {
    webrtc::scoped_refptr<webrtc::I420Buffer> scaled_buffer =
        webrtc::I420Buffer::Create(scaled_width_, scaled_height_);
    int32_t buffer_width = ((scaled_width_ + 15) / 16) * 16;
    int32_t buffer_height = ((scaled_height_ + 15) / 16) * 16;

    NvBufSurfaceAllocateParams input_params = {0};
    input_params.params.width = buffer_width;
    input_params.params.height = buffer_height;
    input_params.params.layout = NVBUF_LAYOUT_PITCH;
    input_params.params.colorFormat = NVBUF_COLOR_FORMAT_YUV420;
    input_params.params.memType = NVBUF_MEM_SURFACE_ARRAY;
    input_params.memtag = NvBufSurfaceTag_NONE;

    NvBufSurface* dst_surf = 0;

    if (NvBufSurfaceAllocate(
            &dst_surf,
            1, /* NvUtils では複数のバッファーを同時に初期化できるため、バッファーの数を指定する */
            &input_params) == -1) {
      RTC_LOG(LS_ERROR) << __FUNCTION__ << " Failed to NvBufSurfaceAllocate";
      return scaled_buffer;
    }
    NvBufSurfaceParams params = dst_surf->surfaceList[0];

    NvBufSurfTransformRect src_rect, dest_rect;
    src_rect.top = 0;
    src_rect.left = 0;
    src_rect.width = params.width;
    src_rect.height = params.height;
    dest_rect.top = 0;
    dest_rect.left = 0;
    dest_rect.width = buffer_width;
    dest_rect.height = buffer_height;

    NvBufSurfTransformParams trans_params;
    memset(&trans_params, 0, sizeof(trans_params));
    trans_params.transform_flag = NVBUFSURF_TRANSFORM_FILTER;
    trans_params.transform_flip = NvBufSurfTransform_None;
    trans_params.transform_filter = NvBufSurfTransformInter_Algo3;
    trans_params.src_rect = &src_rect;
    trans_params.dst_rect = &dest_rect;

    NvBufSurface* src_surf = 0;
    if (NvBufSurfaceFromFd(fd_, (void**)(&src_surf)) == -1) {
      RTC_LOG(LS_ERROR) << __FUNCTION__ << " Failed to NvBufSurfaceFromFd";
      return scaled_buffer;
    }

    if (NvBufSurfTransform(src_surf, dst_surf, &trans_params) !=
        NvBufSurfTransformError_Success) {
      RTC_LOG(LS_ERROR) << __FUNCTION__ << " Failed to NvBufSurfTransform";
      return scaled_buffer;
    }

    int ret;
    void* data_addr;
    uint8_t* dest_addr;
    int num_planes = dst_surf->surfaceList->planeParams.num_planes;
    int index = 0;
    for (int plane = 0; plane < num_planes; plane++) {
      ret = NvBufSurfaceMap(dst_surf, index, plane, NVBUF_MAP_READ);
      if (ret == 0) {
        NvBufSurfaceSyncForCpu(dst_surf, index, plane);
        data_addr = dst_surf->surfaceList->mappedAddr.addr[plane];
        int height, width;
        if (plane == 0) {
          dest_addr = scaled_buffer.get()->MutableDataY();
          width = scaled_width_;
          height = scaled_height_;
        } else if (plane == 1) {
          dest_addr = scaled_buffer.get()->MutableDataU();
          width = (scaled_width_ + 1) >> 1;
          height = (scaled_height_ + 1) >> 1;
        } else if (plane == 2) {
          dest_addr = scaled_buffer.get()->MutableDataV();
          width = (scaled_width_ + 1) >> 1;
          height = (scaled_height_ + 1) >> 1;
        }
        for (int i = 0; i < height; i++) {
          memcpy(dest_addr + width * i,
                 (uint8_t*)data_addr +
                     dst_surf->surfaceList->planeParams.pitch[plane] * i,
                 width);
        }
      }
      NvBufSurfaceUnMap(dst_surf, index, plane);
      if (ret == -1) {
        RTC_LOG(LS_ERROR) << __FUNCTION__
                          << " Failed to NvBufSurfaceMap plane=" << plane;
        return scaled_buffer;
      }
    }

    NvBufSurfaceDestroy(dst_surf);

    return scaled_buffer;
  } else {
    webrtc::scoped_refptr<webrtc::I420Buffer> i420_buffer =
        webrtc::I420Buffer::Create(raw_width_, raw_height_);
    const int conversionResult = libyuv::ConvertToI420(
        data_.get(), length_, i420_buffer.get()->MutableDataY(),
        i420_buffer.get()->StrideY(), i420_buffer.get()->MutableDataU(),
        i420_buffer.get()->StrideU(), i420_buffer.get()->MutableDataV(),
        i420_buffer.get()->StrideV(), 0, 0, raw_width_, raw_height_, raw_width_,
        raw_height_, libyuv::kRotate0, ConvertVideoType(video_type_));
    if (raw_width_ == scaled_width_ && raw_height_ == scaled_height_) {
      return i420_buffer;
    }
    webrtc::scoped_refptr<webrtc::I420Buffer> scaled_buffer =
        webrtc::I420Buffer::Create(scaled_width_, scaled_height_);
    scaled_buffer->ScaleFrom(*i420_buffer->ToI420());
    return scaled_buffer;
  }
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

std::shared_ptr<JetsonJpegDecoder> JetsonBuffer::JpegDecoder() const {
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

JetsonBuffer::JetsonBuffer(webrtc::VideoType video_type,
                           int raw_width,
                           int raw_height,
                           int scaled_width,
                           int scaled_height,
                           int fd,
                           uint32_t pixfmt,
                           std::shared_ptr<JetsonJpegDecoder> decoder)
    : video_type_(video_type),
      raw_width_(raw_width),
      raw_height_(raw_height),
      scaled_width_(scaled_width),
      scaled_height_(scaled_height),
      fd_(fd),
      pixfmt_(pixfmt),
      decoder_(decoder),
      data_(nullptr) {}

JetsonBuffer::JetsonBuffer(webrtc::VideoType video_type,
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
      data_(static_cast<uint8_t*>(webrtc::AlignedMalloc(
          webrtc::CalcBufferSize(video_type, raw_width, raw_height),
          kBufferAlignment))) {}

}  // namespace sora
