#include "jetson_jpeg_decoder_pool.h"

// WebRTC
#include <rtc_base/logging.h>

std::shared_ptr<JetsonJpegDecoder> JetsonJpegDecoderPool::Pop() {
  std::unique_ptr<NvJPEGDecoder> nv_decoder;

  // プールを使うとなぜか実行時にクラッシュすることがあるのでコメントアウト
  // 多分 nvjpeg のバグ
  {
    //std::lock_guard<std::mutex> lock(mtx_);
    //if (decoder_queue_.size() == 0) {
    nv_decoder.reset(NvJPEGDecoder::createJPEGDecoder("jpegdec"));
    //} else {
    //  nv_decoder = std::move(decoder_queue_.front());
    //  decoder_queue_.pop();
    //}
  }

  std::shared_ptr<JetsonJpegDecoder> decoder(
      new JetsonJpegDecoder(shared_from_this(), std::move(nv_decoder)));
  return decoder;
}

void JetsonJpegDecoderPool::Push(std::unique_ptr<NvJPEGDecoder> decoder) {
  std::lock_guard<std::mutex> lock(mtx_);
  //decoder_queue_.push(std::move(decoder));
}