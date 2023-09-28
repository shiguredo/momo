#include "v4l2_buffers.h"

#include <unistd.h>

// Linux
#include <sys/ioctl.h>
#include <sys/mman.h>

// WebRTC
#include <modules/video_coding/include/video_error_codes.h>
#include <rtc_base/logging.h>

int V4L2Buffers::Allocate(int fd,
                          int type,
                          int memory,
                          int req_count,
                          v4l2_format* format,
                          bool export_dmafds) {
  Deallocate();

  v4l2_requestbuffers reqbufs = {};
  reqbufs.count = req_count;
  reqbufs.type = type;
  reqbufs.memory = memory;
  if (ioctl(fd, VIDIOC_REQBUFS, &reqbufs) < 0) {
    RTC_LOG(LS_ERROR) << __FUNCTION__ << "  Failed to request buffers";
    return WEBRTC_VIDEO_CODEC_ERROR;
  }
  buffers_.resize(reqbufs.count);
  RTC_LOG(LS_INFO) << "Request buffers: type=" << reqbufs.type
                   << " memory=" << reqbufs.memory << " count=" << reqbufs.count
                   << " capabilities=" << reqbufs.capabilities;

  if (memory == V4L2_MEMORY_MMAP) {
    for (unsigned int i = 0; i < reqbufs.count; i++) {
      v4l2_plane planes[VIDEO_MAX_PLANES];
      v4l2_buffer v4l2_buf = {};
      v4l2_buf.type = type;
      v4l2_buf.memory = V4L2_MEMORY_MMAP;
      v4l2_buf.index = i;
      v4l2_buf.length = 1;
      v4l2_buf.m.planes = planes;
      if (ioctl(fd, VIDIOC_QUERYBUF, &v4l2_buf) < 0) {
        RTC_LOG(LS_ERROR) << __FUNCTION__ << "  Failed to query buffer"
                          << "  index: " << i;
        return WEBRTC_VIDEO_CODEC_ERROR;
      }

      Buffer buffer;
      buffer.n_planes = v4l2_buf.length;
      for (size_t j = 0; j < buffer.n_planes; j++) {
        PlaneBuffer& plane = buffer.planes[j];
        if (!export_dmafds) {
          plane.start =
              mmap(0, v4l2_buf.m.planes[j].length, PROT_READ | PROT_WRITE,
                   MAP_SHARED, fd, v4l2_buf.m.planes[j].m.mem_offset);
          if (plane.start == MAP_FAILED) {
            RTC_LOG(LS_ERROR)
                << __FUNCTION__ << "  Failed to map plane buffer"
                << "  buffer index: " << i << "  plane index: " << j;
            return WEBRTC_VIDEO_CODEC_ERROR;
          }
          plane.length = v4l2_buf.m.planes[j].length;
        } else {
          v4l2_exportbuffer expbuf = {};
          expbuf.type = type;
          expbuf.index = i;
          expbuf.plane = j;
          if (ioctl(fd, VIDIOC_EXPBUF, &expbuf) < 0) {
            RTC_LOG(LS_ERROR) << __FUNCTION__ << "  Failed to export buffer"
                              << " index=" << i << " plane=" << j
                              << " error=" << strerror(errno);
            return WEBRTC_VIDEO_CODEC_ERROR;
          }
          plane.fd = expbuf.fd;
        }
        plane.sizeimage = format->fmt.pix_mp.plane_fmt[j].sizeimage;
        plane.bytesperline = format->fmt.pix_mp.plane_fmt[j].bytesperline;
      }
      buffers_[i] = buffer;
    }
  }
  fd_ = fd;
  type_ = type;
  memory_ = memory;
  export_dmafds_ = export_dmafds;

  return WEBRTC_VIDEO_CODEC_OK;
}

void V4L2Buffers::Deallocate() {
  for (int i = 0; i < buffers_.size(); i++) {
    Buffer& buffer = buffers_[i];
    for (size_t j = 0; j < buffer.n_planes; j++) {
      PlaneBuffer* plane = &buffer.planes[j];
      if (plane->start != nullptr) {
        if (munmap(plane->start, plane->length) < 0) {
          RTC_LOG(LS_ERROR) << __FUNCTION__ << "  Failed to unmap buffer"
                            << "  index: " << i;
        }
      }
      if (plane->fd != 0) {
        close(plane->fd);
      }
    }
  }
  buffers_.clear();

  if (fd_ != 0) {
    v4l2_requestbuffers reqbufs = {};
    reqbufs.count = 0;
    reqbufs.type = type_;
    reqbufs.memory = memory_;
    if (ioctl(fd_, VIDIOC_REQBUFS, &reqbufs) < 0) {
      RTC_LOG(LS_ERROR) << __FUNCTION__ << "  Failed to free buffers: error="
                        << strerror(errno);
    }
    fd_ = 0;
    type_ = 0;
    memory_ = 0;
  }
}

V4L2Buffers::~V4L2Buffers() {
  Deallocate();
}

int V4L2Buffers::type() const {
  return type_;
}
int V4L2Buffers::memory() const {
  return memory_;
}
int V4L2Buffers::count() const {
  return buffers_.size();
}
bool V4L2Buffers::dmafds_exported() const {
  return export_dmafds_;
}
V4L2Buffers::Buffer& V4L2Buffers::at(int index) {
  return buffers_.at(index);
}
