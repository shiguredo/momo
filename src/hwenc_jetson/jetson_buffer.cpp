#include "jetson_buffer.h"

JetsonBuffer::JetsonBuffer(int fd,
                           uint64_t timestamp_us,
                           std::unique_ptr<NvJPEGDecoder> decoder)
      : fd_(fd),
        timestamp_us_(timestamp_us),
        decoder_(std::move(decoder)) {
}

int JetsonBuffer::GetFd() {
  return fd_;
}

uint64_t JetsonBuffer::GetTimestampUs() {
  return timestamp_us_;
}