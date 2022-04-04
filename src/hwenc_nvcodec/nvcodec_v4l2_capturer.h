#ifndef NVCODEC_V4L2_CAPTURER_H_
#define NVCODEC_V4L2_CAPTURER_H_

#include "v4l2_video_capturer/v4l2_video_capturer.h"

#include "./nvcodec_decoder_cuda.h"

struct NvCodecV4L2CapturerConfig : V4L2VideoCapturerConfig {
  NvCodecV4L2CapturerConfig(const V4L2VideoCapturerConfig& config) {
    *static_cast<V4L2VideoCapturerConfig*>(this) = config;
  }
  std::shared_ptr<CudaContext> cuda_context;
};

class NvCodecV4L2Capturer : public V4L2VideoCapturer {
 public:
  static rtc::scoped_refptr<V4L2VideoCapturer> Create(
      NvCodecV4L2CapturerConfig config);

  bool UseNativeBuffer() override;

 private:
  static rtc::scoped_refptr<V4L2VideoCapturer> Create(
      webrtc::VideoCaptureModule::DeviceInfo* device_info,
      NvCodecV4L2CapturerConfig config,
      size_t capture_device_index);

  void OnCaptured(uint8_t* data, uint32_t bytesused) override;

  std::shared_ptr<NvCodecDecoderCuda> decoder_;
};

#endif  // NVCODEC_V4L2_CAPTURER_H_
