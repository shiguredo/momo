#ifndef SORA_HWENC_JETSON_JETSON_JPEG_DECODER_H_
#define SORA_HWENC_JETSON_JETSON_JPEG_DECODER_H_

#include <memory>

#include "jetson_jpeg_decoder_pool.h"

class NvJPEGDecoder;

namespace sora {

class JetsonJpegDecoderPool;

class JetsonJpegDecoder {
 public:
  JetsonJpegDecoder(std::shared_ptr<JetsonJpegDecoderPool> pool,
                    std::shared_ptr<NvJPEGDecoder> decoder);
  ~JetsonJpegDecoder();

  int DecodeToFd(int& fd,
                 unsigned char* in_buf,
                 unsigned long in_buf_size,
                 uint32_t& pixfmt,
                 uint32_t& width,
                 uint32_t& height);

 private:
  std::shared_ptr<JetsonJpegDecoderPool> pool_;
  std::shared_ptr<NvJPEGDecoder> decoder_;
};

}  // namespace sora

#endif