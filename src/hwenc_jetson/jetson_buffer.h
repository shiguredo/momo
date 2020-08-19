#ifndef JETSON_BUFFER_H_
#define JETSON_BUFFER_H_

#include <memory>
#include "NvJpegDecoder.h"

class JetsonBuffer {
 public:
  JetsonBuffer(int fd,
               uint64_t timestamp_us,
               std::unique_ptr<NvJPEGDecoder> decoder);

  int GetFd();
  uint64_t GetTimestampUs();

 private:
  int fd_;
  uint64_t timestamp_us_;
  std::unique_ptr<NvJPEGDecoder> decoder_;
};
#endif  // JETSON_BUFFER_H_
