#include "sora/hwenc_nvcodec/nvcodec_v4l2_capturer.h"

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>

// WebRTC
#include <api/make_ref_counted.h>
#include <api/scoped_refptr.h>
#include <api/video/nv12_buffer.h>
#include <api/video/video_frame.h>
#include <api/video/video_frame_buffer.h>
#include <api/video/video_rotation.h>
#include <modules/video_capture/video_capture.h>
#include <modules/video_capture/video_capture_factory.h>
#include <rtc_base/logging.h>
#include <rtc_base/time_utils.h>

#include "sora/cuda_context.h"
#include "sora/hwenc_nvcodec/nvcodec_decoder_cuda.h"
#include "sora/v4l2/v4l2_video_capturer.h"

namespace sora {

NvCodecV4L2Capturer::NvCodecV4L2Capturer(
    const NvCodecV4L2CapturerConfig& config)
    : V4L2VideoCapturer(config), config_(config) {}

webrtc::scoped_refptr<NvCodecV4L2Capturer> NvCodecV4L2Capturer::Create(
    const NvCodecV4L2CapturerConfig& config) {
  return webrtc::make_ref_counted<NvCodecV4L2Capturer>(config);
}

int32_t NvCodecV4L2Capturer::Init() {
  decoder_.reset(
      new NvCodecDecoderCuda(config_.cuda_context, CudaVideoCodec::JPEG));

  return V4L2VideoCapturer::Init();
}

void NvCodecV4L2Capturer::OnCaptured(uint8_t* data, uint32_t bytesused) {
  const int64_t timestamp_us = webrtc::TimeMicros();
  int adapted_width, adapted_height, crop_width, crop_height, crop_x, crop_y;
  if (!AdaptFrame(_currentWidth, _currentHeight, timestamp_us, &adapted_width,
                  &adapted_height, &crop_width, &crop_height, &crop_x,
                  &crop_y)) {
    return;
  }

  auto count = decoder_->Decode(data, bytesused);
  for (int i = 0; i < count; i++) {
    // 頑張ればここのコピーは無くせるけど、ひとまずは妥協しておく
    auto buffer = webrtc::NV12Buffer::Create(_currentWidth, _currentHeight);
    uint8_t* p = decoder_->GetFrame();
    //RTC_LOG(LS_INFO) << "_currentWidth=" << _currentWidth
    //                 << " _currentHeight=" << _currentHeight
    //                 << " height=" << buffer->height()
    //                 << " StrideY=" << buffer->StrideY()
    //                 << " ChromaHeight=" << buffer->ChromaHeight()
    //                 << " StrideUV=" << buffer->StrideUV();
    std::memcpy(buffer->MutableDataY(), p,
                buffer->height() * buffer->StrideY() +
                    buffer->ChromaHeight() * buffer->StrideUV());

    webrtc::scoped_refptr<webrtc::VideoFrameBuffer> buf;
    if (_currentWidth != adapted_width || _currentHeight != adapted_height) {
      buf = buffer->CropAndScale(crop_x, crop_y, crop_width, crop_height,
                                 adapted_width, adapted_height);
    } else {
      buf = buffer;
    }

    OnFrame(webrtc::VideoFrame::Builder()
                .set_video_frame_buffer(buf)
                .set_timestamp_rtp(0)
                .set_timestamp_ms(webrtc::TimeMillis())
                .set_timestamp_us(webrtc::TimeMicros())
                .set_rotation(webrtc::kVideoRotation_0)
                .build());
  }
}

}  // namespace sora
