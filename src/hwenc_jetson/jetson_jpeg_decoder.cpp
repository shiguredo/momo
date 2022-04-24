#include "jetson_jpeg_decoder.h"

JetsonJpegDecoder::JetsonJpegDecoder(
    std::shared_ptr<JetsonJpegDecoderPool> pool,
    std::unique_ptr<NvJPEGDecoder> decoder)
    : pool_(pool), decoder_(std::move(decoder)) {}

JetsonJpegDecoder::~JetsonJpegDecoder() {
  pool_->Push(std::move(decoder_));
}

int JetsonJpegDecoder::DecodeToFd(int& fd,
                                  unsigned char* in_buf,
                                  unsigned long in_buf_size,
                                  uint32_t& pixfmt,
                                  uint32_t& width,
                                  uint32_t& height) {
  return decoder_->decodeToFd(fd, in_buf, in_buf_size, pixfmt, width, height);
}