#ifndef SORA_HWENC_NVCODEC_NVCODEC_VIDEO_DECODER_H_
#define SORA_HWENC_NVCODEC_NVCODEC_VIDEO_DECODER_H_

// WebRTC
#include <api/video_codecs/video_decoder.h>
#include <common_video/include/video_frame_buffer_pool.h>
#include <rtc_base/platform_thread.h>

#include "nvcodec_decoder_cuda.h"
#include "sora/cuda_context.h"

namespace sora {

class NvCodecVideoDecoder : public webrtc::VideoDecoder {
 public:
  NvCodecVideoDecoder(std::shared_ptr<CudaContext> context,
                      CudaVideoCodec codec);
  ~NvCodecVideoDecoder() override;

  static bool IsSupported(std::shared_ptr<CudaContext> context,
                          CudaVideoCodec codec);

  bool Configure(const Settings& settings) override;

  int32_t Decode(const webrtc::EncodedImage& input_image,
                 bool missing_frames,
                 int64_t render_time_ms) override;

  int32_t RegisterDecodeCompleteCallback(
      webrtc::DecodedImageCallback* callback) override;

  int32_t Release() override;

  const char* ImplementationName() const override;

 private:
  bool InitNvCodec();
  void ReleaseNvCodec();

  webrtc::DecodedImageCallback* decode_complete_callback_ = nullptr;
  webrtc::VideoFrameBufferPool buffer_pool_;

  std::shared_ptr<CudaContext> context_;
  CudaVideoCodec codec_;
  std::unique_ptr<NvCodecDecoderCuda> decoder_;
  bool output_info_ = false;
};

}  // namespace sora

#endif
