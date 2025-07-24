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
    : V4L2VideoCapturer(config) {}

webrtc::scoped_refptr<V4L2VideoCapturer> NvCodecV4L2Capturer::Create(
    const NvCodecV4L2CapturerConfig& config) {
  webrtc::scoped_refptr<V4L2VideoCapturer> capturer;
  std::unique_ptr<webrtc::VideoCaptureModule::DeviceInfo> device_info(
      webrtc::VideoCaptureFactory::CreateDeviceInfo());
  if (!device_info) {
    RTC_LOG(LS_ERROR) << "Failed to CreateDeviceInfo";
    return nullptr;
  }

  LogDeviceList(device_info.get());

  for (int i = 0; i < device_info->NumberOfDevices(); ++i) {
    capturer = Create(device_info.get(), config, i);
    if (capturer) {
      RTC_LOG(LS_INFO) << "Get Capture";
      return capturer;
    }
  }
  RTC_LOG(LS_ERROR) << "Failed to create NvCodecV4L2Capturer";
  return nullptr;
}

webrtc::scoped_refptr<V4L2VideoCapturer> NvCodecV4L2Capturer::Create(
    webrtc::VideoCaptureModule::DeviceInfo* device_info,
    const NvCodecV4L2CapturerConfig& config,
    size_t capture_device_index) {
  char device_name[256];
  char unique_name[256];

  if (device_info->GetDeviceName(static_cast<uint32_t>(capture_device_index),
                                 device_name, sizeof(device_name), unique_name,
                                 sizeof(unique_name)) != 0) {
    RTC_LOG(LS_WARNING) << "Failed to GetDeviceName";
    return nullptr;
  }
  // config.video_device が指定されている場合は、デバイス名かユニーク名と一致する必要がある
  if (!(config.video_device.empty() || config.video_device == device_name ||
        config.video_device == unique_name)) {
    return nullptr;
  }

  webrtc::scoped_refptr<NvCodecV4L2Capturer> v4l2_capturer =
      webrtc::make_ref_counted<NvCodecV4L2Capturer>(config);

  v4l2_capturer->decoder_.reset(
      new NvCodecDecoderCuda(config.cuda_context, CudaVideoCodec::JPEG));

  if (v4l2_capturer->Init((const char*)&unique_name) < 0) {
    RTC_LOG(LS_WARNING) << "Failed to create NvCodecV4L2Capturer("
                        << unique_name << ")";
    return nullptr;
  }

  if (v4l2_capturer->StartCapture(config) < 0) {
    RTC_LOG(LS_WARNING) << "Failed to start NvCodecV4L2Capturer(w = "
                        << config.width << ", h = " << config.height
                        << ", fps = " << config.framerate << ")";
    return nullptr;
  }

  return v4l2_capturer;
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
