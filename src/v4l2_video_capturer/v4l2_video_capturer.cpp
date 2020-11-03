/*
 *  Copyright (c) 2012 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "v4l2_video_capturer.h"

// C
#include <stdio.h>
#include <string.h>
#include <time.h>

// C++
#include <new>
#include <string>

// Linux
#include <errno.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/select.h>
#include <unistd.h>

// WebRTC
#include <api/scoped_refptr.h>
#include <api/video/i420_buffer.h>
#include <media/base/video_common.h>
#include <modules/video_capture/video_capture.h>
#include <modules/video_capture/video_capture_factory.h>
#include <rtc/native_buffer.h>
#include <rtc_base/logging.h>
#include <rtc_base/ref_counted_object.h>
#include <third_party/libyuv/include/libyuv.h>

rtc::scoped_refptr<V4L2VideoCapturer> V4L2VideoCapturer::Create(
    V4L2VideoCapturerConfig config) {
  rtc::scoped_refptr<V4L2VideoCapturer> capturer;
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
  RTC_LOG(LS_ERROR) << "Failed to create V4L2VideoCapturer";
  return nullptr;
}

void V4L2VideoCapturer::LogDeviceList(
    webrtc::VideoCaptureModule::DeviceInfo* device_info) {
  for (int i = 0; i < device_info->NumberOfDevices(); ++i) {
    char device_name[256];
    char unique_name[256];
    if (device_info->GetDeviceName(static_cast<uint32_t>(i), device_name,
                                   sizeof(device_name), unique_name,
                                   sizeof(unique_name)) != 0) {
      RTC_LOG(LS_WARNING) << "Failed to GetDeviceName(" << i << ")";
      continue;
    }
    RTC_LOG(LS_INFO) << "GetDeviceName(" << i
                     << "): device_name=" << device_name
                     << ", unique_name=" << unique_name;
  }
}

rtc::scoped_refptr<V4L2VideoCapturer> V4L2VideoCapturer::Create(
    webrtc::VideoCaptureModule::DeviceInfo* device_info,
    V4L2VideoCapturerConfig config,
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
      new rtc::RefCountedObject<V4L2VideoCapturer>());
  if (v4l2_capturer->Init((const char*)&unique_name, config.video_device) < 0) {
    RTC_LOG(LS_WARNING) << "Failed to create V4L2VideoCapturer(" << unique_name
                        << ")";
    return nullptr;
  }
  if (v4l2_capturer->StartCapture(config) < 0) {
    RTC_LOG(LS_WARNING) << "Failed to start V4L2VideoCapturer(w = "
                        << config.width << ", h = " << config.height
                        << ", fps = " << config.framerate << ")";
    return nullptr;
  }
  return v4l2_capturer;
}

V4L2VideoCapturer::V4L2VideoCapturer()
    : _deviceFd(-1),
      _buffersAllocatedByDevice(-1),
      _currentWidth(-1),
      _currentHeight(-1),
      _currentFrameRate(-1),
      _useNative(false),
      _captureStarted(false),
      _captureVideoType(webrtc::VideoType::kI420),
      _pool(NULL) {}

bool V4L2VideoCapturer::FindDevice(const char* deviceUniqueIdUTF8,
                                   const std::string& device) {
  int fd;
  if ((fd = open(device.c_str(), O_RDONLY)) != -1) {
    // query device capabilities
    struct v4l2_capability cap;
    if (ioctl(fd, VIDIOC_QUERYCAP, &cap) == 0) {
      if (cap.bus_info[0] != 0) {
        if (strncmp((const char*)cap.bus_info, (const char*)deviceUniqueIdUTF8,
                    strlen((const char*)deviceUniqueIdUTF8)) ==
            0)  // match with device id
        {
          close(fd);
          return true;
        }
      }
    }
    close(fd);  // close since this is not the matching device
  }
  return false;
}

int32_t V4L2VideoCapturer::Init(const char* deviceUniqueIdUTF8,
                                const std::string& specifiedVideoDevice) {
  int fd;
  bool found = false;

  if (!specifiedVideoDevice.empty()) {
    // specifiedVideoDevice が指定されてる場合はそれだけ調べる
    if (FindDevice(deviceUniqueIdUTF8, specifiedVideoDevice)) {
      found = true;
      _videoDevice = specifiedVideoDevice;
    }
  } else {
    // specifiedVideoDevice が指定されてない場合は頑張って探す
    /* detect /dev/video [0-63] entries */
    char device[32];
    int n;
    for (n = 0; n < 64; n++) {
      sprintf(device, "/dev/video%d", n);
      if (FindDevice(deviceUniqueIdUTF8, device)) {
        found = true;
        _videoDevice = device;  // store the video device
        break;
      }
    }
  }

  if (!found) {
    RTC_LOG(LS_INFO) << "no matching device found";
    return -1;
  }
  return 0;
}

V4L2VideoCapturer::~V4L2VideoCapturer() {
  StopCapture();
  if (_deviceFd != -1)
    close(_deviceFd);
}

int32_t V4L2VideoCapturer::StartCapture(V4L2VideoCapturerConfig config) {
  if (_captureStarted) {
    if (config.width == _currentWidth && config.height == _currentHeight) {
      return 0;
    } else {
      StopCapture();
    }
  }

  webrtc::MutexLock lock(&capture_lock_);
  // first open /dev/video device
  if ((_deviceFd = open(_videoDevice.c_str(), O_RDWR | O_NONBLOCK, 0)) < 0) {
    RTC_LOG(LS_INFO) << "error in opening " << _videoDevice
                     << " errono = " << errno;
    return -1;
  }

  // Supported video formats in preferred order.
  // If the requested resolution is larger than VGA, we prefer MJPEG. Go for
  // I420 otherwise.
  const int nFormats = 7;
  unsigned int fmts[nFormats] = {};
  if (config.use_native) {
    fmts[0] = V4L2_PIX_FMT_MJPEG;
    fmts[1] = V4L2_PIX_FMT_JPEG;
  } else {
    fmts[0] = V4L2_PIX_FMT_YUV420;
    fmts[1] = V4L2_PIX_FMT_YVU420;
    fmts[2] = V4L2_PIX_FMT_NV12;
    fmts[3] = V4L2_PIX_FMT_YUYV;
    fmts[4] = V4L2_PIX_FMT_UYVY;
    fmts[5] = V4L2_PIX_FMT_MJPEG;
    fmts[6] = V4L2_PIX_FMT_JPEG;
  }

  // Enumerate image formats.
  struct v4l2_fmtdesc fmt;
  int fmtsIdx = nFormats;
  memset(&fmt, 0, sizeof(fmt));
  fmt.index = 0;
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  RTC_LOG(LS_INFO) << "Video Capture enumerats supported image formats:";
  while (ioctl(_deviceFd, VIDIOC_ENUM_FMT, &fmt) == 0) {
    RTC_LOG(LS_INFO) << "  { pixelformat = "
                     << cricket::GetFourccName(fmt.pixelformat)
                     << ", description = '" << fmt.description << "' }";
    // Match the preferred order.
    for (int i = 0; i < nFormats; i++) {
      if (fmt.pixelformat == fmts[i] && i < fmtsIdx)
        fmtsIdx = i;
    }
    // Keep enumerating.
    fmt.index++;
  }

  if (fmtsIdx == nFormats) {
    RTC_LOG(LS_INFO) << "no supporting video formats found";
    return -1;
  } else {
    RTC_LOG(LS_INFO) << "We prefer format "
                     << cricket::GetFourccName(fmts[fmtsIdx]);
  }

  struct v4l2_format video_fmt;
  memset(&video_fmt, 0, sizeof(struct v4l2_format));
  video_fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  video_fmt.fmt.pix.sizeimage = 0;
  video_fmt.fmt.pix.width = config.width;
  video_fmt.fmt.pix.height = config.height;
  video_fmt.fmt.pix.pixelformat = fmts[fmtsIdx];

  if (video_fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_YUYV)
    _captureVideoType = webrtc::VideoType::kYUY2;
  else if (video_fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_YUV420)
    _captureVideoType = webrtc::VideoType::kI420;
  else if (video_fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_YVU420)
    _captureVideoType = webrtc::VideoType::kYV12;
  else if (video_fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_NV12)
    _captureVideoType = webrtc::VideoType::kNV12;
  else if (video_fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_UYVY)
    _captureVideoType = webrtc::VideoType::kUYVY;
  else if (video_fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_MJPEG ||
           video_fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_JPEG)
    _captureVideoType = webrtc::VideoType::kMJPEG;

  // set format and frame size now
  if (ioctl(_deviceFd, VIDIOC_S_FMT, &video_fmt) < 0) {
    RTC_LOG(LS_INFO) << "error in VIDIOC_S_FMT, errno = " << errno;
    return -1;
  }

  // initialize current width and height
  _currentWidth = video_fmt.fmt.pix.width;
  _currentHeight = video_fmt.fmt.pix.height;

  // Trying to set frame rate, before check driver capability.
  bool driver_framerate_support = true;
  struct v4l2_streamparm streamparms;
  memset(&streamparms, 0, sizeof(streamparms));
  streamparms.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (ioctl(_deviceFd, VIDIOC_G_PARM, &streamparms) < 0) {
    RTC_LOG(LS_INFO) << "error in VIDIOC_G_PARM errno = " << errno;
    driver_framerate_support = false;
    // continue
  } else {
    // check the capability flag is set to V4L2_CAP_TIMEPERFRAME.
    if (streamparms.parm.capture.capability & V4L2_CAP_TIMEPERFRAME) {
      // driver supports the feature. Set required framerate.
      memset(&streamparms, 0, sizeof(streamparms));
      streamparms.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      streamparms.parm.capture.timeperframe.numerator = 1;
      streamparms.parm.capture.timeperframe.denominator = config.framerate;
      if (ioctl(_deviceFd, VIDIOC_S_PARM, &streamparms) < 0) {
        RTC_LOG(LS_INFO) << "Failed to set the framerate. errno=" << errno;
        driver_framerate_support = false;
      } else {
        _currentFrameRate = config.framerate;
      }
    }
  }
  // If driver doesn't support framerate control, need to hardcode.
  // Hardcoding the value based on the frame size.
  if (!driver_framerate_support) {
    if (!_useNative && _currentWidth >= 800 &&
        _captureVideoType != webrtc::VideoType::kMJPEG) {
      _currentFrameRate = 15;
    } else {
      _currentFrameRate = 30;
    }
  }

  if (!AllocateVideoBuffers()) {
    RTC_LOG(LS_INFO) << "failed to allocate video capture buffers";
    return -1;
  }

  // start capture thread;
  if (!_captureThread) {
    quit_ = false;
    _captureThread.reset(
        new rtc::PlatformThread(V4L2VideoCapturer::CaptureThread, this,
                                "CaptureThread", rtc::kHighPriority));
    _captureThread->Start();
  }

  // Needed to start UVC camera - from the uvcview application
  enum v4l2_buf_type type;
  type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (ioctl(_deviceFd, VIDIOC_STREAMON, &type) == -1) {
    RTC_LOG(LS_INFO) << "Failed to turn on stream";
    return -1;
  }

  _useNative = config.use_native;
  _captureStarted = true;
  return 0;
}

int32_t V4L2VideoCapturer::StopCapture() {
  if (_captureThread) {
    {
      webrtc::MutexLock lock(&capture_lock_);
      quit_ = true;
    }
    // Make sure the capture thread stop stop using the capture_lock_.
    _captureThread->Stop();
    _captureThread.reset();
  }

  webrtc::MutexLock lock(&capture_lock_);
  if (_captureStarted) {
    _captureStarted = false;

    DeAllocateVideoBuffers();
    close(_deviceFd);
    _deviceFd = -1;
  }

  return 0;
}

bool V4L2VideoCapturer::UseNativeBuffer() {
  return _useNative && (_captureVideoType == webrtc::VideoType::kMJPEG ||
                        _captureVideoType == webrtc::VideoType::kI420);
}

// critical section protected by the caller

bool V4L2VideoCapturer::AllocateVideoBuffers() {
  struct v4l2_requestbuffers rbuffer;
  memset(&rbuffer, 0, sizeof(v4l2_requestbuffers));

  rbuffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  rbuffer.memory = V4L2_MEMORY_MMAP;
  rbuffer.count = kNoOfV4L2Bufffers;

  if (ioctl(_deviceFd, VIDIOC_REQBUFS, &rbuffer) < 0) {
    RTC_LOG(LS_INFO) << "Could not get buffers from device. errno = " << errno;
    return false;
  }

  if (rbuffer.count > kNoOfV4L2Bufffers)
    rbuffer.count = kNoOfV4L2Bufffers;

  _buffersAllocatedByDevice = rbuffer.count;

  // Map the buffers
  _pool = new Buffer[rbuffer.count];

  for (unsigned int i = 0; i < rbuffer.count; i++) {
    struct v4l2_buffer buffer;
    memset(&buffer, 0, sizeof(v4l2_buffer));
    buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buffer.memory = V4L2_MEMORY_MMAP;
    buffer.index = i;

    if (ioctl(_deviceFd, VIDIOC_QUERYBUF, &buffer) < 0) {
      return false;
    }

    _pool[i].start = mmap(NULL, buffer.length, PROT_READ | PROT_WRITE,
                          MAP_SHARED, _deviceFd, buffer.m.offset);

    if (MAP_FAILED == _pool[i].start) {
      for (unsigned int j = 0; j < i; j++)
        munmap(_pool[j].start, _pool[j].length);
      return false;
    }

    _pool[i].length = buffer.length;

    if (ioctl(_deviceFd, VIDIOC_QBUF, &buffer) < 0) {
      return false;
    }
  }
  return true;
}

bool V4L2VideoCapturer::DeAllocateVideoBuffers() {
  // unmap buffers
  for (int i = 0; i < _buffersAllocatedByDevice; i++)
    munmap(_pool[i].start, _pool[i].length);

  delete[] _pool;

  // turn off stream
  enum v4l2_buf_type type;
  type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (ioctl(_deviceFd, VIDIOC_STREAMOFF, &type) < 0) {
    RTC_LOG(LS_INFO) << "VIDIOC_STREAMOFF error. errno: " << errno;
  }

  return true;
}

void V4L2VideoCapturer::CaptureThread(void* obj) {
  V4L2VideoCapturer* capturer = static_cast<V4L2VideoCapturer*>(obj);
  while (capturer->CaptureProcess()) {
  }
}

bool V4L2VideoCapturer::CaptureProcess() {
  int retVal = 0;
  fd_set rSet;
  struct timeval timeout;

  FD_ZERO(&rSet);
  FD_SET(_deviceFd, &rSet);
  timeout.tv_sec = 1;
  timeout.tv_usec = 0;

  // _deviceFd written only in StartCapture, when this thread isn't running.
  retVal = select(_deviceFd + 1, &rSet, NULL, NULL, &timeout);
  if (retVal < 0 && errno != EINTR)  // continue if interrupted
  {
    // select failed
    return false;
  } else if (retVal == 0) {
    // select timed out
    return true;
  } else if (!FD_ISSET(_deviceFd, &rSet)) {
    // not event on camera handle
    return true;
  }

  {
    webrtc::MutexLock lock(&capture_lock_);

    if (quit_) {
      return false;
    }

    if (_captureStarted) {
      struct v4l2_buffer buf;
      memset(&buf, 0, sizeof(struct v4l2_buffer));
      buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_MMAP;
      // dequeue a buffer - repeat until dequeued properly!
      while (ioctl(_deviceFd, VIDIOC_DQBUF, &buf) < 0) {
        if (errno != EINTR) {
          RTC_LOG(LS_INFO) << "could not sync on a buffer on device "
                           << strerror(errno);
          return true;
        }
      }

      if (!OnCaptured(buf)) {
        // enqueue the buffer again
        if (ioctl(_deviceFd, VIDIOC_QBUF, &buf) == -1) {
          RTC_LOG(LS_INFO) << __FUNCTION__
                           << " Failed to enqueue capture buffer";
        }
      }
    }
  }
  usleep(0);
  return true;
}

bool V4L2VideoCapturer::OnCaptured(struct v4l2_buffer& buf) {
  rtc::scoped_refptr<webrtc::VideoFrameBuffer> dst_buffer = nullptr;
  if (UseNativeBuffer()) {
    rtc::scoped_refptr<NativeBuffer> native_buffer(
        NativeBuffer::Create(_captureVideoType, _currentWidth, _currentHeight));
    memcpy(native_buffer->MutableData(), (unsigned char*)_pool[buf.index].start,
           buf.bytesused);
    native_buffer->SetLength(buf.bytesused);
    dst_buffer = native_buffer;
  } else {
    rtc::scoped_refptr<webrtc::I420Buffer> i420_buffer(
        webrtc::I420Buffer::Create(_currentWidth, _currentHeight));
    i420_buffer->InitializeData();
    if (libyuv::ConvertToI420(
            (unsigned char*)_pool[buf.index].start, buf.bytesused,
            i420_buffer.get()->MutableDataY(), i420_buffer.get()->StrideY(),
            i420_buffer.get()->MutableDataU(), i420_buffer.get()->StrideU(),
            i420_buffer.get()->MutableDataV(), i420_buffer.get()->StrideV(), 0,
            0, _currentWidth, _currentHeight, _currentWidth, _currentHeight,
            libyuv::kRotate0, ConvertVideoType(_captureVideoType)) < 0) {
      RTC_LOG(LS_ERROR) << "ConvertToI420 Failed";
    } else {
      dst_buffer = i420_buffer;
    }
  }

  if (dst_buffer) {
    webrtc::VideoFrame video_frame = webrtc::VideoFrame::Builder()
                                         .set_video_frame_buffer(dst_buffer)
                                         .set_timestamp_rtp(0)
                                         .set_timestamp_ms(rtc::TimeMillis())
                                         .set_timestamp_us(rtc::TimeMicros())
                                         .set_rotation(webrtc::kVideoRotation_0)
                                         .build();
    OnCapturedFrame(video_frame);
  }

  // enqueue the buffer again
  if (ioctl(_deviceFd, VIDIOC_QBUF, &buf) == -1) {
    RTC_LOG(LS_INFO) << "Failed to enqueue capture buffer";
  }
  return true;
}
