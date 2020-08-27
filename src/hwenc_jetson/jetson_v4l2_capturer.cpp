/*
 *  Copyright (c) 2012 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "jetson_v4l2_capturer.h"

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

// Jetson Linux Multimedia API
#include <nvbuf_utils.h>
#include <NvJpegDecoder.h>

#include "jetson_buffer.h"

#define MJPEG_EOS_SEARCH_SIZE 4096

rtc::scoped_refptr<JetsonV4L2Capturer> JetsonV4L2Capturer::Create(
    ConnectionSettings cs) {
  rtc::scoped_refptr<JetsonV4L2Capturer> capturer;
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
  RTC_LOG(LS_ERROR) << "Failed to create JetsonV4L2Capturer";
  return nullptr;
}

void JetsonV4L2Capturer::LogDeviceList(
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

rtc::scoped_refptr<JetsonV4L2Capturer> JetsonV4L2Capturer::Create(
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
  rtc::scoped_refptr<JetsonV4L2Capturer> v4l2_capturer(
      new rtc::RefCountedObject<JetsonV4L2Capturer>());
  if (v4l2_capturer->Init((const char*)&unique_name, cs.video_device) < 0) {
    RTC_LOG(LS_WARNING) << "Failed to create JetsonV4L2Capturer(" << unique_name
                        << ")";
    return nullptr;
  }
  if (v4l2_capturer->StartCapture(cs) < 0) {
    auto size = cs.GetSize();
    RTC_LOG(LS_WARNING) << "Failed to start JetsonV4L2Capturer(w = "
                        << size.width << ", h = " << size.height
                        << ", fps = " << cs.framerate << ")";
    return nullptr;
  }
  return v4l2_capturer;
}

JetsonV4L2Capturer::JetsonV4L2Capturer()
    : _deviceFd(-1),
      _buffersAllocatedByDevice(-1),
      _currentWidth(-1),
      _currentHeight(-1),
      _currentFrameRate(-1),
      _useNative(false),
      _captureStarted(false),
      _captureVideoType(webrtc::VideoType::kI420),
      _pool(NULL) {}

bool JetsonV4L2Capturer::FindDevice(const char* deviceUniqueIdUTF8,
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

int32_t JetsonV4L2Capturer::Init(const char* deviceUniqueIdUTF8,
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

JetsonV4L2Capturer::~JetsonV4L2Capturer() {
  StopCapture();
  if (_deviceFd != -1)
    close(_deviceFd);
}

int32_t JetsonV4L2Capturer::StartCapture(ConnectionSettings cs) {
  auto size = cs.GetSize();
  if (_captureStarted) {
    if (size.width == _currentWidth && size.height == _currentHeight) {
      return 0;
    } else {
      StopCapture();
    }
  }

  rtc::CritScope critScope(&_captureCritSect);
  // first open /dev/video device
  if ((_deviceFd = open(_videoDevice.c_str(), O_RDWR | O_NONBLOCK, 0)) < 0) {
    RTC_LOG(LS_INFO) << "error in opening " << _videoDevice
                     << " errono = " << errno;
    return -1;
  }

  // Supported video formats in preferred order.
  // If the requested resolution is larger than VGA, we prefer MJPEG. Go for
  // I420 otherwise.
  const int nFormats = 5;
  unsigned int fmts[nFormats];
  if (!cs.force_i420 && (size.width > 640 || size.height > 480)) {
    fmts[0] = V4L2_PIX_FMT_MJPEG;
    fmts[1] = V4L2_PIX_FMT_YUYV;
    fmts[2] = V4L2_PIX_FMT_YUYV;
    fmts[3] = V4L2_PIX_FMT_UYVY;
    fmts[4] = V4L2_PIX_FMT_JPEG;
  } else {
    fmts[0] = V4L2_PIX_FMT_YUYV;
    fmts[1] = V4L2_PIX_FMT_YUYV;
    fmts[2] = V4L2_PIX_FMT_UYVY;
    fmts[3] = V4L2_PIX_FMT_MJPEG;
    fmts[4] = V4L2_PIX_FMT_JPEG;
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
  video_fmt.fmt.pix.width = size.width;
  video_fmt.fmt.pix.height = size.height;
  video_fmt.fmt.pix.pixelformat = fmts[fmtsIdx];

  if (video_fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_YUYV)
    _captureVideoType = webrtc::VideoType::kYUY2;
  else if (video_fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_YUV420)
    _captureVideoType = webrtc::VideoType::kI420;
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
      streamparms.parm.capture.timeperframe.denominator = cs.framerate;
      if (ioctl(_deviceFd, VIDIOC_S_PARM, &streamparms) < 0) {
        RTC_LOG(LS_INFO) << "Failed to set the framerate. errno=" << errno;
        driver_framerate_support = false;
      } else {
        _currentFrameRate = cs.framerate;
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
        new rtc::PlatformThread(JetsonV4L2Capturer::CaptureThread, this,
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

  _useNative = cs.use_native;
  _captureStarted = true;
  return 0;
}

int32_t JetsonV4L2Capturer::StopCapture() {
  if (_captureThread) {
    {
      rtc::CritScope cs(&_captureCritSect);
      quit_ = true;
    }
    // Make sure the capture thread stop stop using the critsect.
    _captureThread->Stop();
    _captureThread.reset();
  }

  rtc::CritScope cs(&_captureCritSect);
  if (_captureStarted) {
    _captureStarted = false;

    DeAllocateVideoBuffers();
    close(_deviceFd);
    _deviceFd = -1;
  }

  return 0;
}

static NvBufferColorFormat GetNvbuffColorFmt(
    webrtc::VideoType captureVideoType) {
  if (captureVideoType == webrtc::VideoType::kMJPEG) {
    return NvBufferColorFormat_YUV420;
  } else if (captureVideoType == webrtc::VideoType::kI420) {
    return NvBufferColorFormat_YUV420;
  } else if (captureVideoType == webrtc::VideoType::kYUY2) {
    return NvBufferColorFormat_YUYV;
  } else if (captureVideoType == webrtc::VideoType::kUYVY) {
    return NvBufferColorFormat_UYVY;
  }
  return NvBufferColorFormat_Invalid;
}

bool JetsonV4L2Capturer::UseNativeBuffer() {
  return true;
}

// critical section protected by the caller
// _pool を初期化してしまっていることを除いて request_camera_buff_mmap と同等
bool JetsonV4L2Capturer::AllocateVideoBuffers() {
  uint32_t buff_memory_type = V4L2_MEMORY_MMAP;
  if (_captureVideoType != webrtc::VideoType::kMJPEG) {
    buff_memory_type = V4L2_MEMORY_DMABUF;
  }

  struct v4l2_requestbuffers rbuffer;
  memset(&rbuffer, 0, sizeof(v4l2_requestbuffers));

  rbuffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  rbuffer.memory = buff_memory_type;
  rbuffer.count = kNoOfV4L2Bufffers;

  if (ioctl(_deviceFd, VIDIOC_REQBUFS, &rbuffer) < 0) {
    RTC_LOG(LS_INFO) << "Could not get buffers from device. errno = " << errno;
    return false;
  }

  if (rbuffer.count > kNoOfV4L2Bufffers)
    rbuffer.count = kNoOfV4L2Bufffers;

  _buffersAllocatedByDevice = rbuffer.count;

  // Map the buffers
  // ここでの初期化は邪魔になるかも
  _pool = new Buffer[rbuffer.count];

  for (unsigned int i = 0; i < rbuffer.count; i++) {
    struct v4l2_buffer buffer;
    memset(&buffer, 0, sizeof(v4l2_buffer));
    buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buffer.memory = buff_memory_type;
    buffer.index = i;

    if (ioctl(_deviceFd, VIDIOC_QUERYBUF, &buffer) < 0) {
      return false;
    }

    if (buff_memory_type == V4L2_MEMORY_MMAP) {
      _pool[i].dmabuff_fd = 0;
      _pool[i].start = mmap(NULL, buffer.length, PROT_READ | PROT_WRITE,
                            MAP_SHARED, _deviceFd, buffer.m.offset);

      if (MAP_FAILED == _pool[i].start) {
        for (unsigned int j = 0; j < i; j++)
          munmap(_pool[j].start, _pool[j].length);
        return false;
      }

      _pool[i].length = buffer.length;
    } else {
      int fd;
      NvBufferCreateParams inputParams = {0};
      inputParams.payloadType = NvBufferPayload_SurfArray;
      inputParams.width = _currentWidth;
      inputParams.height = _currentHeight;
      inputParams.layout = NvBufferLayout_Pitch;
      inputParams.colorFormat = GetNvbuffColorFmt(_captureVideoType);
      inputParams.nvbuf_tag = NvBufferTag_CAMERA;
      if (NvBufferCreateEx(&fd, &inputParams) == -1) {
        RTC_LOG(LS_ERROR) << "Failed to NvBufferCreateEx";
        return false;
      }

      _pool[i].dmabuff_fd = fd;

      if (NvBufferMemMap(_pool[i].dmabuff_fd, 0, NvBufferMem_Read_Write,
                         (void**)&_pool[i].start) == -1) {
        RTC_LOG(LS_ERROR) << "Failed to NvBufferMemMap";
        return false;
      }
      
      buffer.m.fd = (unsigned long)fd;
      if (buffer.length != _pool[i].length)
      {
        _pool[i].length = buffer.length;
      }
    }

    if (ioctl(_deviceFd, VIDIOC_QBUF, &buffer) < 0) {
      return false;
    }
  }
  return true;
}

bool JetsonV4L2Capturer::DeAllocateVideoBuffers() {
  // unmap buffers
  for (int i = 0; i < _buffersAllocatedByDevice; i++) {
    if (_pool[i].dmabuff_fd)
      NvBufferDestroy(_pool[i].dmabuff_fd);
    if (_captureVideoType == webrtc::VideoType::kMJPEG)
      munmap(_pool[i].start, _pool[i].length);
  }

  delete[] _pool;

  // turn off stream
  enum v4l2_buf_type type;
  type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (ioctl(_deviceFd, VIDIOC_STREAMOFF, &type) < 0) {
    RTC_LOG(LS_INFO) << "VIDIOC_STREAMOFF error. errno: " << errno;
  }

  return true;
}

void JetsonV4L2Capturer::CaptureThread(void* obj) {
  JetsonV4L2Capturer* capturer = static_cast<JetsonV4L2Capturer*>(obj);
  while (capturer->CaptureProcess()) {
  }
}

bool JetsonV4L2Capturer::CaptureProcess() {
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
    rtc::CritScope cs(&_captureCritSect);

    if (quit_) {
      return false;
    }

    if (_captureStarted) {
      struct v4l2_buffer buf;
      memset(&buf, 0, sizeof(struct v4l2_buffer));
      buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      if (_captureVideoType == webrtc::VideoType::kMJPEG) {
        buf.memory = V4L2_MEMORY_MMAP;
      } else {
        buf.memory = V4L2_MEMORY_DMABUF;
      }
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

bool JetsonV4L2Capturer::OnCaptured(struct v4l2_buffer& buf) {
  const int64_t timestamp_us = rtc::TimeMicros();
  int adapted_width, adapted_height, crop_width, crop_height, crop_x, crop_y;
  if (!AdaptFrame(_currentWidth, _currentHeight, timestamp_us, &adapted_width,
                  &adapted_height, &crop_width, &crop_height, &crop_x,
                  &crop_y)) {
    return false;
  }

  if (_captureVideoType == webrtc::VideoType::kMJPEG) {
    unsigned int bytesused = buf.bytesused;
    unsigned int eosSearchSize = MJPEG_EOS_SEARCH_SIZE;
    uint8_t *p;
    /* v4l2_buf.bytesused may have padding bytes for alignment
        Search for EOF to get exact size */
    if (eosSearchSize > bytesused)
        eosSearchSize = bytesused;
    for (unsigned int i = 0; i < eosSearchSize; i++) {
        p = (uint8_t *)_pool[buf.index].start + bytesused;
        if ((*(p-2) == 0xff) && (*(p-1) == 0xd9)) {
            break;
        }
        bytesused--;
    }

    std::unique_ptr<NvJPEGDecoder> decoder(NvJPEGDecoder::createJPEGDecoder("jpegdec"));
    int fd = 0;
    uint32_t width, height, pixfmt;
    if (decoder->decodeToFd(fd, (unsigned char *)_pool[buf.index].start,
        bytesused, pixfmt, width, height) < 0) {
      RTC_LOG(LS_ERROR) << "decodeToFd Failed";
      return false;
    }

    rtc::scoped_refptr<JetsonBuffer> jetson_buffer(
        JetsonBuffer::Create(
            pixfmt, width, height, adapted_width, adapted_height,
            fd, std::move(decoder)));
    OnFrame(webrtc::VideoFrame::Builder()
                .set_video_frame_buffer(jetson_buffer)
                .set_timestamp_rtp(0)
                .set_timestamp_ms(rtc::TimeMillis())
                .set_timestamp_us(rtc::TimeMicros())
                .set_rotation(webrtc::kVideoRotation_0)
                .build());

    // enqueue the buffer again
    if (ioctl(_deviceFd, VIDIOC_QBUF, &buf) == -1) {
      RTC_LOG(LS_INFO) << "Failed to enqueue capture buffer";
    }
  } else {
    NvBufferMemSyncForDevice(_pool[buf.index].dmabuff_fd, 0,
            (void**)&_pool[buf.index].start);

    uint32_t pixfmt;
    if (_captureVideoType == webrtc::VideoType::kYUY2)
      pixfmt = V4L2_PIX_FMT_YUYV;
    else if (_captureVideoType == webrtc::VideoType::kI420)
      pixfmt = V4L2_PIX_FMT_YUV420M;
    else if (_captureVideoType == webrtc::VideoType::kUYVY)
      pixfmt = V4L2_PIX_FMT_UYVY;
    else {
      RTC_LOG(LS_ERROR) << " Unsupported pixel format";
      return false;
    }

    rtc::scoped_refptr<JetsonBuffer> jetson_buffer(
        JetsonBuffer::Create(
            pixfmt, _currentWidth, _currentHeight,
            adapted_width, adapted_height,
            _pool[buf.index].dmabuff_fd, _deviceFd, &buf));
    OnFrame(webrtc::VideoFrame::Builder()
                .set_video_frame_buffer(jetson_buffer)
                .set_timestamp_rtp(0)
                .set_timestamp_ms(rtc::TimeMillis())
                .set_timestamp_us(rtc::TimeMicros())
                .set_rotation(webrtc::kVideoRotation_0)
                .build());
  }
  return true;
}
