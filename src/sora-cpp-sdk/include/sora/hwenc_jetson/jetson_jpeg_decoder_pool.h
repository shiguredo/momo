#ifndef SORA_HWENC_JETSON_JETSON_JPEG_DECODER_POOL_H_
#define SORA_HWENC_JETSON_JETSON_JPEG_DECODER_POOL_H_

#include <memory>
#include <mutex>
#include <queue>

#include "jetson_jpeg_decoder.h"

class NvJPEGDecoder;

namespace sora {

class JetsonJpegDecoder;

class JetsonJpegDecoderPool
    : public std::enable_shared_from_this<JetsonJpegDecoderPool> {
 public:
  std::shared_ptr<JetsonJpegDecoder> Pop();
  void Push(std::shared_ptr<NvJPEGDecoder> decoder);

 private:
  std::mutex mtx_;
  std::queue<std::shared_ptr<NvJPEGDecoder>> decoder_queue_;
};

}  // namespace sora

#endif
