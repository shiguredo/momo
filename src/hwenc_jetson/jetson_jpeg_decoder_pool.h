#ifndef JETSON_JPEG_DECODER_POOL_H_
#define JETSON_JPEG_DECODER_POOL_H_

#include <memory>
#include <mutex>
#include <queue>

// Jetson Linux Multimedia API
#include <NvJpegDecoder.h>

#include "jetson_jpeg_decoder.h"

class JetsonJpegDecoder;

class JetsonJpegDecoderPool :
    public std::enable_shared_from_this<JetsonJpegDecoderPool> {
 public:
  std::shared_ptr<JetsonJpegDecoder> Pop();
  void Push(std::unique_ptr<NvJPEGDecoder> decoder);

 private:
  std::mutex mtx_;
  std::queue<std::unique_ptr<NvJPEGDecoder>> decoder_queue_;
};
#endif  // JETSON_JPEG_DECODER_POOL_H_