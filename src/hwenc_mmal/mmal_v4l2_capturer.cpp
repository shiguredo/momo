#include "mmal_v4l2_capturer.h"

// Linux
#include <linux/videodev2.h>
#include <sys/ioctl.h>

// WebRTC
#include <modules/video_capture/video_capture_factory.h>
#include <rtc_base/logging.h>
#include <rtc_base/ref_counted_object.h>

#include "mmal_buffer.h"

rtc::scoped_refptr<V4L2VideoCapturer> MMALV4L2Capturer::Create(
    ConnectionSettings cs) {
  rtc::scoped_refptr<V4L2VideoCapturer> capturer;
  std::unique_ptr<webrtc::VideoCaptureModule::DeviceInfo> device_info(
      webrtc::VideoCaptureFactory::CreateDeviceInfo());
  if (!device_info) {
    RTC_LOG(LS_ERROR) << "Failed to CreateDeviceInfo";
    return nullptr;
  }

  LogDeviceList(device_info.get());

  for (int i = 0; i < device_info->NumberOfDevices(); ++i) {
    capturer = Create(device_info.get(), cs, i);
    if (capturer) {
      RTC_LOG(LS_INFO) << "Get Capture";
      return capturer;
    }
  }
  RTC_LOG(LS_ERROR) << "Failed to create V4L2VideoCapturer";
  return nullptr;
}

rtc::scoped_refptr<V4L2VideoCapturer> MMALV4L2Capturer::Create(
    webrtc::VideoCaptureModule::DeviceInfo* device_info,
    ConnectionSettings cs,
    size_t capture_device_index) {
  char device_name[256];
  char unique_name[256];
  if (device_info->GetDeviceName(static_cast<uint32_t>(capture_device_index),
                                 device_name, sizeof(device_name), unique_name,
                                 sizeof(unique_name)) != 0) {
    RTC_LOG(LS_WARNING) << "Failed to GetDeviceName";
    return nullptr;
  }
  rtc::scoped_refptr<V4L2VideoCapturer> v4l2_capturer(
      new rtc::RefCountedObject<MMALV4L2Capturer>());
  if (v4l2_capturer->Init((const char*)&unique_name, cs.video_device) < 0) {
    RTC_LOG(LS_WARNING) << "Failed to create MMALV4L2Capturer(" << unique_name
                        << ")";
    return nullptr;
  }
  if (v4l2_capturer->StartCapture(cs) < 0) {
    auto size = cs.GetSize();
    RTC_LOG(LS_WARNING) << "Failed to start MMALV4L2Capturer(w = " << size.width
                        << ", h = " << size.height << ", fps = " << cs.framerate
                        << ")";
    return nullptr;
  }
  return v4l2_capturer;
}

MMALV4L2Capturer::MMALV4L2Capturer()
    : component_in_(nullptr),
      decoder_(nullptr),
      resizer_(nullptr),
      connection_(nullptr),
      configured_width_(0),
      configured_height_(0) {
  bcm_host_init();
  decoded_buffer_num_ = 4;
  decoded_buffer_size_ =
      webrtc::CalcBufferSize(webrtc::VideoType::kI420, VCOS_ALIGN_UP(1920, 32),
                             VCOS_ALIGN_UP(1080, 16));
  // 出力のプールはまとめないと、リサイズ時にエンコーダに送ったフレームが破棄される場合がある
  resizer_pool_out_ =
      mmal_pool_create(decoded_buffer_num_, decoded_buffer_size_);
}

MMALV4L2Capturer::~MMALV4L2Capturer() {
  std::lock_guard<std::mutex> lock(mtx_);
  MMALRelease();
  mmal_pool_destroy(resizer_pool_out_);
}

int32_t MMALV4L2Capturer::StartCapture(ConnectionSettings cs) {
  return V4L2VideoCapturer::StartCapture(cs);
}

int32_t MMALV4L2Capturer::StopCapture() {
  return V4L2VideoCapturer::StopCapture();
}

bool MMALV4L2Capturer::UseNativeBuffer() {
  return true;
}

bool MMALV4L2Capturer::OnCaptured(struct v4l2_buffer& buf) {
  const int64_t timestamp_us = rtc::TimeMicros();

  int adapted_width;
  int adapted_height;
  int crop_width;
  int crop_height;
  int crop_x;
  int crop_y;
  if (!AdaptFrame(_currentWidth, _currentHeight, timestamp_us, &adapted_width,
                  &adapted_height, &crop_width, &crop_height, &crop_x,
                  &crop_y)) {
    return false;
  }
  std::lock_guard<std::mutex> lock(mtx_);

  if (configured_width_ != adapted_width ||
      configured_height_ != adapted_height) {
    RTC_LOG(LS_INFO) << "Resizer reinitialized from " << configured_width_
                     << "x" << configured_height_ << " to " << adapted_width
                     << "x" << adapted_height;
    MMALRelease();
    if (MMALConfigure(adapted_width, adapted_height) == -1) {
      RTC_LOG(LS_ERROR) << "Failed to MMALConfigure";
      return false;
    }
  }

  ResizerFillBuffer();

  {
    rtc::CritScope lock(&frame_params_lock_);
    frame_params_.push(absl::make_unique<FrameParams>(
        configured_width_, configured_height_, timestamp_us));
  }

  MMAL_BUFFER_HEADER_T* buffer;
  while ((buffer = mmal_queue_get(pool_in_->queue)) != nullptr) {
    buffer->pts = buffer->dts = timestamp_us;
    buffer->offset = 0;
    buffer->flags = MMAL_BUFFER_HEADER_FLAG_FRAME;
    buffer->data = (uint8_t*)_pool[buf.index].start;
    buffer->length = buffer->alloc_size = buf.bytesused;
    if (mmal_port_send_buffer(component_in_->input[0], buffer) !=
        MMAL_SUCCESS) {
      RTC_LOG(LS_ERROR) << "Failed to send input buffer";
      return false;
    }
  }

  // enqueue the buffer again
  if (ioctl(_deviceFd, VIDIOC_QBUF, &buf) == -1) {
    RTC_LOG(LS_INFO) << "Failed to enqueue capture buffer";
  }
  return true;
}

void MMALV4L2Capturer::MMALInputCallbackFunction(MMAL_PORT_T* port,
                                                 MMAL_BUFFER_HEADER_T* buffer) {
  ((MMALV4L2Capturer*)port->userdata)->MMALInputCallback(port, buffer);
}

void MMALV4L2Capturer::MMALInputCallback(MMAL_PORT_T* port,
                                         MMAL_BUFFER_HEADER_T* buffer) {
  mmal_buffer_header_release(buffer);
}

void MMALV4L2Capturer::ResizerFillBuffer() {
  MMAL_BUFFER_HEADER_T* resizer_out;
  while ((resizer_out = mmal_queue_get(resizer_pool_out_->queue)) != NULL) {
    mmal_port_send_buffer(resizer_->output[0], resizer_out);
  }
}

void MMALV4L2Capturer::ResizerOutputCallbackFunction(
    MMAL_PORT_T* port,
    MMAL_BUFFER_HEADER_T* buffer) {
  ((MMALV4L2Capturer*)port->userdata)->ResizerOutputCallback(port, buffer);
}

void MMALV4L2Capturer::ResizerOutputCallback(MMAL_PORT_T* port,
                                             MMAL_BUFFER_HEADER_T* buffer) {
  std::unique_ptr<FrameParams> params;
  {
    rtc::CritScope lock(&frame_params_lock_);
    do {
      if (frame_params_.empty()) {
        RTC_LOG(LS_WARNING)
            << __FUNCTION__
            << "Frame parameter is not found. SkipFrame pts:" << buffer->pts;
        mmal_buffer_header_release(buffer);
        return;
      }
      params = std::move(frame_params_.front());
      frame_params_.pop();
    } while (params->timestamp < buffer->pts);
    if (params->timestamp != buffer->pts) {
      RTC_LOG(LS_WARNING) << __FUNCTION__
                          << "Frame parameter is not found. SkipFrame pts:"
                          << buffer->pts;
      mmal_buffer_header_release(buffer);
      return;
    }
  }

  rtc::scoped_refptr<MMALBuffer> mmal_buffer(
      MMALBuffer::Create(buffer, params->width, params->height));
  OnFrame(webrtc::VideoFrame::Builder()
              .set_video_frame_buffer(mmal_buffer)
              .set_timestamp_rtp(0)
              .set_timestamp_us(buffer->pts)
              .set_rotation(webrtc::kVideoRotation_0)
              .build());
}

int32_t MMALV4L2Capturer::MMALConfigure(int32_t width, int32_t height) {
  MMAL_PORT_T* port_in;
  MMAL_FOURCC_T input_format;

  if (mmal_component_create("vc.ril.isp", &resizer_) != MMAL_SUCCESS) {
    RTC_LOG(LS_ERROR) << "Failed to create mmal resizer";
    Release();
    return -1;
  }

  if (_captureVideoType == webrtc::VideoType::kMJPEG) {
    if (mmal_component_create(MMAL_COMPONENT_DEFAULT_VIDEO_DECODER,
                              &decoder_) != MMAL_SUCCESS) {
      RTC_LOG(LS_ERROR) << "Failed to create mmal decoder";
      Release();
      return -1;
    }
    component_in_ = decoder_;
    port_in = decoder_->input[0];
    input_format = MMAL_ENCODING_MJPEG;
  } else {
    switch (_captureVideoType) {
      case webrtc::VideoType::kI420:
        input_format = MMAL_ENCODING_I420;
        break;
      case webrtc::VideoType::kUYVY:
        input_format = MMAL_ENCODING_UYVY;
        break;
      case webrtc::VideoType::kYUY2:
        input_format = MMAL_ENCODING_YUYV;
        break;
      default:
        RTC_LOG(LS_ERROR) << "This video type is not supported in native "
                             "frame.  video type : "
                          << _captureVideoType;
        Release();
        return -1;
    }
    component_in_ = resizer_;
    port_in = resizer_->input[0];
    port_in->format->es->video.crop.x = 0;
    port_in->format->es->video.crop.y = 0;
    port_in->format->es->video.crop.width = _currentWidth;
    port_in->format->es->video.crop.height = _currentHeight;
  }

  port_in->format->type = MMAL_ES_TYPE_VIDEO;
  port_in->format->encoding = input_format;
  port_in->format->es->video.width = _currentWidth;
  port_in->format->es->video.height = _currentHeight;

  port_in->buffer_size = port_in->buffer_size_recommended;
  if (port_in->buffer_size < port_in->buffer_size_min)
    port_in->buffer_size = port_in->buffer_size_min;
  if (_captureVideoType == webrtc::VideoType::kMJPEG)
    port_in->buffer_size = 512 << 10;
  // port_in->buffer_num を 2 にあげた方が VPX では 22fps レートは出るが H264 が詰まる
  // port_in->buffer_num が 1 の時は 15fps 程度
  port_in->buffer_num = 1;
  port_in->userdata = (MMAL_PORT_USERDATA_T*)this;

  if (mmal_port_format_commit(port_in) != MMAL_SUCCESS) {
    RTC_LOG(LS_ERROR) << "Failed to commit input port format";
    return -1;
  }

  if (_captureVideoType == webrtc::VideoType::kMJPEG) {
    MMAL_PORT_T* decoder_port_out = decoder_->output[0];
    mmal_format_copy(decoder_port_out->format, port_in->format);
    decoder_port_out->format->type = MMAL_ES_TYPE_VIDEO;
    decoder_port_out->format->encoding = MMAL_ENCODING_OPAQUE;
    decoder_port_out->format->encoding_variant = MMAL_ENCODING_I420;

    if (mmal_port_format_commit(decoder_port_out) != MMAL_SUCCESS) {
      RTC_LOG(LS_ERROR) << "Failed to commit decoder output port format";
      return -1;
    }

    if (mmal_connection_create(
            &connection_, decoder_->output[0], resizer_->input[0],
            MMAL_CONNECTION_FLAG_TUNNELLING) != MMAL_SUCCESS) {
      RTC_LOG(LS_ERROR) << "Failed to connect decoder to resizer";
      return -1;
    }
  }

  MMAL_PORT_T* resizer_port_out = resizer_->output[0];
  mmal_format_copy(resizer_port_out->format, resizer_->input[0]->format);
  resizer_port_out->format->encoding = MMAL_ENCODING_I420;
  resizer_port_out->format->es->video.width = VCOS_ALIGN_UP(width, 32);
  resizer_port_out->format->es->video.height = VCOS_ALIGN_UP(height, 16);
  resizer_port_out->format->es->video.crop.x = 0;
  resizer_port_out->format->es->video.crop.y = 0;
  resizer_port_out->format->es->video.crop.width = width;
  resizer_port_out->format->es->video.crop.height = height;

  resizer_port_out->buffer_size = decoded_buffer_size_;
  resizer_port_out->buffer_num = decoded_buffer_num_;
  resizer_port_out->userdata = (MMAL_PORT_USERDATA_T*)this;

  if (mmal_port_format_commit(resizer_port_out) != MMAL_SUCCESS) {
    RTC_LOG(LS_ERROR) << "Failed to commit output port format";
    return -1;
  }

  // mmal_pool_create で作った場合 mmal_component_enable 前に mmal_port_enable
  if (mmal_port_enable(resizer_port_out, ResizerOutputCallbackFunction) !=
      MMAL_SUCCESS) {
    RTC_LOG(LS_ERROR) << "Failed to enable resizer output port";
    return -1;
  }

  if (mmal_component_enable(resizer_) != MMAL_SUCCESS) {
    RTC_LOG(LS_ERROR) << "Failed to enable component";
    return -1;
  }

  if (mmal_port_parameter_set_boolean(resizer_port_out,
                                      MMAL_PARAMETER_ZERO_COPY,
                                      MMAL_TRUE) != MMAL_SUCCESS) {
    RTC_LOG(LS_ERROR) << "Failed to set resizer output zero copy";
    return -1;
  }

  if (mmal_port_enable(port_in, MMALInputCallbackFunction) != MMAL_SUCCESS) {
    RTC_LOG(LS_ERROR) << "Failed to enable input port";
    return -1;
  }
  pool_in_ =
      mmal_port_pool_create(port_in, port_in->buffer_num, port_in->buffer_size);

  ResizerFillBuffer();

  if (_captureVideoType == webrtc::VideoType::kMJPEG) {
    if (mmal_component_enable(decoder_) != MMAL_SUCCESS) {
      RTC_LOG(LS_ERROR) << "Failed to enable component";
      return -1;
    }

    if (mmal_port_parameter_set_boolean(decoder_->output[0],
                                        MMAL_PARAMETER_ZERO_COPY,
                                        MMAL_TRUE) != MMAL_SUCCESS) {
      RTC_LOG(LS_ERROR) << "Failed to set decoder output zero copy";
      return -1;
    }

    if (mmal_port_parameter_set_boolean(resizer_->input[0],
                                        MMAL_PARAMETER_ZERO_COPY,
                                        MMAL_TRUE) != MMAL_SUCCESS) {
      RTC_LOG(LS_ERROR) << "Failed to set resizer input zero copy";
      return -1;
    }

    if (mmal_connection_enable(connection_) != MMAL_SUCCESS) {
      RTC_LOG(LS_ERROR) << "Failed to enable connection decoder to resizer";
      return -1;
    }
  }

  configured_width_ = width;
  configured_height_ = height;
  return 0;
}

void MMALV4L2Capturer::MMALRelease() {
  if (resizer_) {
    mmal_component_disable(resizer_);
    if (decoder_) {
      mmal_component_disable(decoder_);
      mmal_port_disable(decoder_->input[0]);
      mmal_port_pool_destroy(decoder_->input[0], pool_in_);
    } else {
      mmal_port_disable(resizer_->input[0]);
      mmal_port_pool_destroy(resizer_->input[0], pool_in_);
    }
    mmal_port_disable(resizer_->output[0]);
  }
  if (connection_) {
    mmal_connection_destroy(connection_);
    connection_ = nullptr;
  }
  if (resizer_) {
    mmal_component_destroy(resizer_);
    resizer_ = nullptr;
  }
  if (decoder_) {
    mmal_component_destroy(decoder_);
    decoder_ = nullptr;
  }
  while (!frame_params_.empty())
    frame_params_.pop();
}
