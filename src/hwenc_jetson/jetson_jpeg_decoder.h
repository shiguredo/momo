#ifndef JETSON_JPEG_DECODER_H_
#define JETSON_JPEG_DECODER_H_

#include <memory>

// Jetson Linux Multimedia API
#include <NvJpegDecoder.h>

#include "jetson_jpeg_decoder_pool.h"

class JetsonJpegDecoderPool;

class JetsonJpegDecoder {
 public:
  JetsonJpegDecoder(std::shared_ptr<JetsonJpegDecoderPool> pool,
                    std::unique_ptr<NvJPEGDecoder> decoder);
  ~JetsonJpegDecoder();

  int DecodeToFd(int &fd,
                 unsigned char *in_buf, unsigned long in_buf_size,
                 uint32_t &pixfmt, uint32_t &width, uint32_t &height);
 private:
  std::shared_ptr<JetsonJpegDecoderPool> pool_;
  std::unique_ptr<NvJPEGDecoder> decoder_;

};
#endif  // JETSON_JPEG_DECODER_H_