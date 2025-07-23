#include "libcamera_capturer.h"

#include <stdint.h>
#include <sys/mman.h>
#include <iostream>

// WebRTC
#include <api/video/i420_buffer.h>
#include <rtc_base/logging.h>
#include <third_party/libyuv/include/libyuv.h>

#include "v4l2_native_buffer.h"

webrtc::scoped_refptr<LibcameraCapturer> LibcameraCapturer::Create(
    LibcameraCapturerConfig config) {
  webrtc::scoped_refptr<LibcameraCapturer> capturer;

  LogDeviceList();

  for (int i = 0; i < 1; ++i) {
    capturer = Create(config, i);
    if (capturer) {
      RTC_LOG(LS_INFO) << "Get Capture";
      return capturer;
    }
  }
  RTC_LOG(LS_ERROR) << "Failed to create LibcameraCapturer";
  return nullptr;
}

void LibcameraCapturer::LogDeviceList() {
  auto camera_manager = libcameracpp_CameraManager_new();
  int ret = libcamerac_CameraManager_start(camera_manager.get());
  if (ret) {
    RTC_LOG(LS_ERROR) << __FUNCTION__
                      << "libcamera CameraManager failed to start.  code: "
                      << ret;
  } else {
    auto cameras = libcameracpp_CameraManager_cameras(camera_manager.get());
    int i = 0;
    if (libcamerac_vector_Camera_size(cameras.get()) != 0) {
      //for (auto const& camera : cameras) {
      //  RTC_LOG(LS_INFO) << "GetDeviceName(" << i++ << "): device_name="
      //                   << camera->properties().get(
      //                          libcamera::properties::Model)
      //                   << ", unique_name=" << camera->id();
      //}
    } else {
      RTC_LOG(LS_ERROR) << "No cameras available";
    }
  }
  libcamerac_CameraManager_stop(camera_manager.get());
}

webrtc::scoped_refptr<LibcameraCapturer> LibcameraCapturer::Create(
    LibcameraCapturerConfig config,
    size_t capture_device_index) {
  webrtc::scoped_refptr<LibcameraCapturer> capturer(
      new webrtc::RefCountedObject<LibcameraCapturer>());
  if (capturer->Init(capture_device_index) < 0) {
    RTC_LOG(LS_WARNING) << "Failed to create LibcameraCapturer("
                        << capture_device_index << ")";
    return nullptr;
  }
  if (capturer->StartCapture(config) < 0) {
    RTC_LOG(LS_WARNING) << "Failed to start LibcameraCapturer(w = "
                        << config.width << ", h = " << config.height
                        << ", fps = " << config.framerate << ")";
    return nullptr;
  }
  return capturer;
}

LibcameraCapturer::LibcameraCapturer()
    : sora::ScalableVideoTrackSource(sora::ScalableVideoTrackSourceConfig()),
      acquired_(false),
      controls_(libcameracpp_ControlList_controls()),
      camera_started_(false) {}

LibcameraCapturer::~LibcameraCapturer() {
  Release();
}

int32_t LibcameraCapturer::Init(int camera_id) {
  auto camera_manager = libcameracpp_CameraManager_new();
  int ret = libcamerac_CameraManager_start(camera_manager.get());
  if (ret) {
    RTC_LOG(LS_ERROR) << __FUNCTION__ << " CameraManager failed to start."
                      << "  code: " << ret;
    return -1;
  }

  auto cameras = libcameracpp_CameraManager_cameras(camera_manager.get());
  auto camera_size = libcamerac_vector_Camera_size(cameras.get());
  if (camera_size == 0) {
    RTC_LOG(LS_ERROR) << __FUNCTION__ << " No cameras available.";
    return -1;
  }
  if (camera_id >= camera_size) {
    RTC_LOG(LS_ERROR) << __FUNCTION__ << " Selected camera id is not available";
    return -1;
  }

  camera_manager_ = camera_manager;
  auto camera = libcamerac_vector_Camera_at(cameras.get(), camera_id);
  std::string cam_id = libcamerac_Camera_id(camera);
  camera_ =
      libcameracpp_CameraManager_get(camera_manager_.get(), cam_id.c_str());
  if (!camera_) {
    RTC_LOG(LS_ERROR) << __FUNCTION__ << " Failed to find camera " << cam_id;
    return -1;
  }

  if (libcamerac_Camera_acquire(camera_.get())) {
    RTC_LOG(LS_ERROR) << __FUNCTION__ << " Failed to acquire camera " << cam_id;
    return -1;
  }
  acquired_ = true;
  return 0;
}

void LibcameraCapturer::Release() {
  if (acquired_)
    libcamerac_Camera_release(camera_.get());
  acquired_ = false;

  camera_.reset();

  camera_manager_.reset();
}

int32_t LibcameraCapturer::StartCapture(LibcameraCapturerConfig config) {
  auto stream_roles = libcameracpp_vector_StreamRole_new();
  libcamerac_vector_StreamRole_push_back(stream_roles.get(),
                                         libcamerac_StreamRole_VideoRecording);
  configuration_ = libcameracpp_Camera_generateConfiguration(
      camera_.get(), stream_roles.get());
  if (!configuration_) {
    RTC_LOG(LS_ERROR) << __FUNCTION__ << " Failed to generateConfiguration";
    return -1;
  }

  auto cfg = libcamerac_CameraConfiguration_at(configuration_.get(), 0);
  libcamerac_StreamConfiguration_set_pixelFormat(
      cfg, libcamerac_PixelFormat_YUV420());
  libcamerac_StreamConfiguration_set_bufferCount(cfg, 6);
  libcamerac_StreamConfiguration_set_size_width(cfg, config.width);
  libcamerac_StreamConfiguration_set_size_height(cfg, config.height);
  //libcamerac_StreamConfiguration_set_colorSpace(cfg,
  //                                              libcamerac_ColorSpace_Jpeg());

  auto validation =
      libcamerac_CameraConfiguration_validate(configuration_.get());
  if (validation == libcamerac_CameraConfiguration_Status_Invalid) {
    RTC_LOG(LS_ERROR) << __FUNCTION__
                      << " Failed to validate stream configurations";
    return -1;
  } else if (validation == libcamerac_CameraConfiguration_Status_Adjusted) {
    RTC_LOG(LS_WARNING) << __FUNCTION__ << " Camera configuration adjusted";
    return -1;
  }

  if (libcamerac_Camera_configure(camera_.get(), configuration_.get()) < 0) {
    RTC_LOG(LS_ERROR) << __FUNCTION__ << " Failed to configure camera";
    return -1;
  }

  allocator_ = libcameracpp_FrameBufferAllocator_new(camera_.get());
  stream_ = libcamerac_StreamConfiguration_stream(cfg);
  if (libcamerac_FrameBufferAllocator_allocate(allocator_.get(), stream_) < 0) {
    RTC_LOG(LS_ERROR) << __FUNCTION__ << " Failed to allocate buffers";
    return -1;
  }

  auto buffers_size =
      libcamerac_FrameBufferAllocator_buffers_size(allocator_.get(), stream_);
  for (int i = 0; i < buffers_size; i++) {
    auto buffer = libcamerac_FrameBufferAllocator_buffers_at(allocator_.get(),
                                                             stream_, i);
    int size = 0;
    int planes_size = libcamerac_FrameBuffer_planes_size(buffer);
    for (int i = 0; i < planes_size; i++) {
      auto plane = libcamerac_FrameBuffer_planes_at(buffer, i);
      auto fd = libcamerac_FrameBuffer_Plane_fd(plane);
      size += libcamerac_FrameBuffer_Plane_length(plane);
      if (i == planes_size - 1 ||
          fd != libcamerac_FrameBuffer_Plane_fd(
                    libcamerac_FrameBuffer_planes_at(buffer, i + 1))) {
        if (config.native_frame_output) {
          mapped_buffers_[buffer].push_back(Span{nullptr, (int)size, fd});
        } else {
          void* memory =
              mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
          mapped_buffers_[buffer].push_back(
              Span{static_cast<uint8_t*>(memory), (int)size, 0});
        }
      }
    }
    frame_buffer_.push(buffer);
  }

  while (true) {
    if (frame_buffer_.empty()) {
      break;
    }
    auto request = libcameracpp_Camera_createRequest(camera_.get());
    if (!request) {
      RTC_LOG(LS_ERROR) << __FUNCTION__ << " Failed to create request";
      return -1;
    }
    requests_.push_back(request);
    libcamerac_FrameBuffer* buffer = frame_buffer_.front();
    frame_buffer_.pop();
    if (libcamerac_Request_addBuffer(requests_.back().get(), stream_, buffer) <
        0) {
      RTC_LOG(LS_ERROR) << __FUNCTION__ << " Failed to add buffer to request";
      return -1;
    }
  }

  if (libcamerac_Camera_start(camera_.get(), controls_.get())) {
    RTC_LOG(LS_ERROR) << __FUNCTION__ << " Failed to start camera";
    return -1;
  }
  libcamerac_ControlList_clear(controls_.get());
  camera_started_ = true;

  auto signal = libcamerac_Camera_requestCompleted(camera_.get());
  libcamerac_Signal_Request_connect(
      signal, &LibcameraCapturer::requestCompleteStatic, this);

  for (auto& request : requests_) {
    if (libcamerac_Camera_queueRequest(camera_.get(), request.get()) < 0) {
      RTC_LOG(LS_ERROR) << __FUNCTION__ << " Failed to queue request";
      return -1;
    }
  }
  return 0;
}

int32_t LibcameraCapturer::StopCapture() {
  {
    std::lock_guard<std::mutex> lock(camera_stop_mutex_);
    if (camera_started_) {
      if (libcamerac_Camera_stop(camera_.get())) {
        RTC_LOG(LS_ERROR) << __FUNCTION__ << " Failed to stop camera";
        return -1;
      }
      camera_started_ = false;
    }
  }

  if (camera_) {
    auto signal = libcamerac_Camera_requestCompleted(camera_.get());
    libcamerac_Signal_Request_disconnect(
        signal, &LibcameraCapturer::requestCompleteStatic, this);
  }

  requests_.clear();

  return 0;
}

void LibcameraCapturer::requestCompleteStatic(libcamerac_Request* request,
                                              void* user_data) {
  auto self = static_cast<LibcameraCapturer*>(user_data);
  self->requestComplete(request);
}

void LibcameraCapturer::requestComplete(libcamerac_Request* request) {
  if (libcamerac_Request_status(request) ==
      libcamerac_Request_Status_RequestCancelled) {
    return;
  }

  auto cfg = libcamerac_CameraConfiguration_at(configuration_.get(), 0);

  int width = libcamerac_StreamConfiguration_get_size_width(cfg);
  int height = libcamerac_StreamConfiguration_get_size_height(cfg);
  int stride = libcamerac_StreamConfiguration_get_stride(cfg);

  const int64_t timestamp_us = webrtc::TimeMicros();
  int adapted_width, adapted_height, crop_width, crop_height, crop_x, crop_y;
  if (!AdaptFrame(width, height, timestamp_us, &adapted_width, &adapted_height,
                  &crop_width, &crop_height, &crop_x, &crop_y)) {
    RTC_LOG(LS_INFO) << "Drop frame";
    queueRequest(request);
    return;
  }

  libcamerac_FrameBuffer* buffer =
      libcamerac_Request_findBuffer(request, stream_);
  auto item = mapped_buffers_.find(buffer);
  if (item == mapped_buffers_.end()) {
    return;
  }
  const std::vector<Span>& buffers = item->second;

  webrtc::scoped_refptr<webrtc::VideoFrameBuffer> frame_buffer;
  if (buffers[0].buffer != nullptr) {
    // メモリ出力なので I420Buffer に格納する
    webrtc::scoped_refptr<webrtc::I420Buffer> i420_buffer(
        webrtc::I420Buffer::Create(adapted_width, adapted_height));
    auto chroma_stride = stride / 2;
    auto chroma_height = (height + 1) / 2;
    auto src_y = buffers[0].buffer;
    auto src_u = src_y + stride * height;
    auto src_v = src_y + stride * height + chroma_stride * chroma_height;
    if (libyuv::I420Scale(src_y, stride, src_u, stride / 2, src_v, stride / 2,
                          width, height, i420_buffer->MutableDataY(),
                          i420_buffer->StrideY(), i420_buffer->MutableDataU(),
                          i420_buffer->StrideU(), i420_buffer->MutableDataV(),
                          i420_buffer->StrideV(), adapted_width, adapted_height,
                          libyuv::kFilterBox) < 0) {
      RTC_LOG(LS_ERROR) << "I420Scale Failed";
    }

    frame_buffer = i420_buffer;

    webrtc::VideoFrame video_frame = webrtc::VideoFrame::Builder()
                                         .set_video_frame_buffer(frame_buffer)
                                         .set_timestamp_rtp(0)
                                         .set_timestamp_us(webrtc::TimeMicros())
                                         .set_rotation(webrtc::kVideoRotation_0)
                                         .build();
    OnFrame(video_frame);
    queueRequest(request);
  } else {
    // DMA なので V4L2NativeBuffer に格納する
    frame_buffer = webrtc::make_ref_counted<V4L2NativeBuffer>(
        webrtc::VideoType::kI420, width, height, adapted_width, adapted_height,
        buffers[0].fd, nullptr, buffers[0].length, stride,
        [this, request]() { queueRequest(request); });
    RTC_LOG(LS_VERBOSE) << "V4L2NativeBuffer created: with=" << width
                        << " height=" << height << " stride=" << stride
                        << " adapted_width=" << adapted_width
                        << " adapted_height=" << adapted_height
                        << " fd=" << buffers[0].fd
                        << " buffers_size=" << buffers.size();

    webrtc::VideoFrame video_frame = webrtc::VideoFrame::Builder()
                                         .set_video_frame_buffer(frame_buffer)
                                         .set_timestamp_rtp(0)
                                         .set_timestamp_us(webrtc::TimeMicros())
                                         .set_rotation(webrtc::kVideoRotation_0)
                                         .build();
    OnFrame(video_frame);
  }
}

void LibcameraCapturer::queueRequest(libcamerac_Request* request) {
  std::map<const libcamerac_Stream*, libcamerac_FrameBuffer*> buffers;
  auto map = libcamerac_Request_buffers(request);
  libcamerac_Request_BufferMap_foreach(
      map,
      [](const libcamerac_Stream* stream, libcamerac_FrameBuffer* buffer,
         void* data) {
        (*((std::map<const libcamerac_Stream*, libcamerac_FrameBuffer*>*)
               data))[stream] = buffer;
      },
      &buffers);
  libcamerac_Request_reuse(request);

  std::lock_guard<std::mutex> stop_lock(camera_stop_mutex_);
  if (!camera_started_) {
    return;
  }

  for (auto const& p : buffers) {
    if (libcamerac_Request_addBuffer(request, p.first, p.second) < 0) {
      RTC_LOG(LS_ERROR) << __FUNCTION__ << " Failed to add buffer to request";
      return;
    }
  }

  libcamerac_ControlList_copy(controls_.get(),
                              libcamerac_Request_controls(request));
  libcamerac_ControlList_clear(controls_.get());

  if (libcamerac_Camera_queueRequest(camera_.get(), request) < 0) {
    RTC_LOG(LS_ERROR) << __FUNCTION__ << " Failed to queue request";
    return;
  }
}
