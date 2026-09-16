#ifndef SORA_HWENC_V4L2_V4L2_BUFFERS_H_
#define SORA_HWENC_V4L2_V4L2_BUFFERS_H_

#include <vector>

// Linux
#include <linux/videodev2.h>

namespace sora {

// MMAP なバッファと DMABUF なバッファを扱うためのクラス
class V4L2Buffers {
 public:
  struct PlaneBuffer {
    void* start = nullptr;
    size_t length = 0;
    int sizeimage = 0;
    int bytesperline = 0;
    int fd = 0;
  };
  struct Buffer {
    PlaneBuffer planes[VIDEO_MAX_PLANES];
    size_t n_planes = 0;
  };

  int Allocate(int fd,
               int type,
               int memory,
               int req_count,
               v4l2_format* format,
               bool export_dmafds);

  void Deallocate();

  ~V4L2Buffers();

  int type() const;
  int memory() const;
  int count() const;
  bool dmafds_exported() const;
  Buffer& at(int index);

 private:
  int fd_ = 0;
  int type_ = 0;
  int memory_ = 0;
  bool export_dmafds_ = false;
  std::vector<Buffer> buffers_;
};

}  // namespace sora

#endif
