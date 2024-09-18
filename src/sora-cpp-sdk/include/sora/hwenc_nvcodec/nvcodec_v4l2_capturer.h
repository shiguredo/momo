#ifndef SORA_HWENC_NVCODEC_NVCODEC_V4L2_CAPTURER_H_
#define SORA_HWENC_NVCODEC_NVCODEC_V4L2_CAPTURER_H_

#include "nvcodec_decoder_cuda.h"
#include "sora/v4l2/v4l2_video_capturer.h"

namespace sora {

struct NvCodecV4L2CapturerConfig : V4L2VideoCapturerConfig {
  NvCodecV4L2CapturerConfig(const V4L2VideoCapturerConfig& config) {
    *static_cast<V4L2VideoCapturerConfig*>(this) = config;
  }
  std::shared_ptr<CudaContext> cuda_context;
};

class NvCodecV4L2Capturer : public V4L2VideoCapturer {
 public:
  static rtc::scoped_refptr<V4L2VideoCapturer> Create(
      const NvCodecV4L2CapturerConfig& config);
  NvCodecV4L2Capturer(const NvCodecV4L2CapturerConfig& config);

 private:
  static rtc::scoped_refptr<V4L2VideoCapturer> Create(
      webrtc::VideoCaptureModule::DeviceInfo* device_info,
      const NvCodecV4L2CapturerConfig& config,
      size_t capture_device_index);

  void OnCaptured(uint8_t* data, uint32_t bytesused) override;

  std::shared_ptr<NvCodecDecoderCuda> decoder_;
};

}  // namespace sora

#endif
