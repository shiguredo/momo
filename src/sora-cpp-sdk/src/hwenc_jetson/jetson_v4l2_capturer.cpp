#include "sora/hwenc_jetson/jetson_v4l2_capturer.h"

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
#include <rtc_base/logging.h>
#include <rtc_base/ref_counted_object.h>
#include <third_party/libyuv/include/libyuv.h>

// L4T Multimedia API
#include <NvBufSurface.h>

#include "sora/hwenc_jetson/jetson_buffer.h"

#define MJPEG_EOS_SEARCH_SIZE 4096

namespace sora {

webrtc::scoped_refptr<JetsonV4L2Capturer> JetsonV4L2Capturer::Create(
    const V4L2VideoCapturerConfig& config) {
  webrtc::scoped_refptr<JetsonV4L2Capturer> capturer;
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

webrtc::scoped_refptr<JetsonV4L2Capturer> JetsonV4L2Capturer::Create(
    webrtc::VideoCaptureModule::DeviceInfo* device_info,
    const V4L2VideoCapturerConfig& config,
    size_t capture_device_index) {
  char device_name[256];
  char unique_name[256];
  if (device_info->GetDeviceName(static_cast<uint32_t>(capture_device_index),
                                 device_name, sizeof(device_name), unique_name,
                                 sizeof(unique_name)) != 0) {
    RTC_LOG(LS_WARNING) << "Failed to GetDeviceName";
    return nullptr;
  }
  webrtc::scoped_refptr<JetsonV4L2Capturer> v4l2_capturer =
      webrtc::make_ref_counted<JetsonV4L2Capturer>(config);
  if (v4l2_capturer->Init((const char*)&unique_name, config.video_device) < 0) {
    RTC_LOG(LS_WARNING) << "Failed to create JetsonV4L2Capturer(" << unique_name
                        << ")";
    return nullptr;
  }
  if (v4l2_capturer->StartCapture(config) < 0) {
    RTC_LOG(LS_WARNING) << "Failed to start JetsonV4L2Capturer(w = "
                        << config.width << ", h = " << config.height
                        << ", fps = " << config.framerate << ")";
    return nullptr;
  }
  return v4l2_capturer;
}

JetsonV4L2Capturer::JetsonV4L2Capturer(const V4L2VideoCapturerConfig& config)
    : ScalableVideoTrackSource(config),
      _deviceFd(-1),
      _buffersAllocatedByDevice(-1),
      _currentWidth(-1),
      _currentHeight(-1),
      _currentFrameRate(-1),
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

int32_t JetsonV4L2Capturer::StartCapture(
    const V4L2VideoCapturerConfig& config) {
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
  const int nFormats = 6;
  unsigned int fmts[nFormats] = {};
  if (config.use_native) {
    fmts[0] = V4L2_PIX_FMT_MJPEG;
    fmts[1] = V4L2_PIX_FMT_JPEG;
  } else if (config.force_yuy2) {
    // --force-yuy2 が指定された場合は YUY2 を優先
    fmts[0] = V4L2_PIX_FMT_YUYV;
    fmts[1] = V4L2_PIX_FMT_UYVY;
    fmts[2] = V4L2_PIX_FMT_YUV420;
    fmts[3] = V4L2_PIX_FMT_YVU420;
    fmts[4] = V4L2_PIX_FMT_MJPEG;
    fmts[5] = V4L2_PIX_FMT_JPEG;
  } else if (!config.force_i420 &&
             (config.width > 640 || config.height > 480)) {
    fmts[0] = V4L2_PIX_FMT_MJPEG;
    fmts[1] = V4L2_PIX_FMT_YUV420;
    fmts[2] = V4L2_PIX_FMT_YVU420;
    fmts[3] = V4L2_PIX_FMT_YUYV;
    fmts[4] = V4L2_PIX_FMT_UYVY;
    fmts[5] = V4L2_PIX_FMT_JPEG;
  } else {
    fmts[0] = V4L2_PIX_FMT_YUV420;
    fmts[1] = V4L2_PIX_FMT_YVU420;
    fmts[2] = V4L2_PIX_FMT_YUYV;
    fmts[3] = V4L2_PIX_FMT_UYVY;
    fmts[4] = V4L2_PIX_FMT_MJPEG;
    fmts[5] = V4L2_PIX_FMT_JPEG;
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
                     << webrtc::GetFourccName(fmt.pixelformat)
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
                     << webrtc::GetFourccName(fmts[fmtsIdx]);
  }

  struct v4l2_format video_fmt;
  memset(&video_fmt, 0, sizeof(struct v4l2_format));
  video_fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  video_fmt.fmt.pix.sizeimage = 0;
  video_fmt.fmt.pix.width = config.width;
  video_fmt.fmt.pix.height = config.height;
  video_fmt.fmt.pix.pixelformat = fmts[fmtsIdx];

  // set format and frame size now
  if (ioctl(_deviceFd, VIDIOC_S_FMT, &video_fmt) < 0) {
    RTC_LOG(LS_INFO) << "error in VIDIOC_S_FMT, errno = " << errno;
    return -1;
  }

  // initialize current width and height
  _currentWidth = video_fmt.fmt.pix.width;
  _currentHeight = video_fmt.fmt.pix.height;
  _currentPixelFormat = video_fmt.fmt.pix.pixelformat;

  if (video_fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_YUYV)
    _captureVideoType = webrtc::VideoType::kYUY2;
  else if (video_fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_YUV420)
    _captureVideoType = webrtc::VideoType::kI420;
  else if (video_fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_YVU420)
    _captureVideoType = webrtc::VideoType::kYV12;
  else if (video_fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_UYVY)
    _captureVideoType = webrtc::VideoType::kUYVY;
  else if (video_fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_MJPEG ||
           video_fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_JPEG)
    _captureVideoType = webrtc::VideoType::kMJPEG;

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
    if (!config.use_native && _currentWidth >= 800 &&
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
  if (_captureThread.empty()) {
    quit_ = false;
    _captureThread = webrtc::PlatformThread::SpawnJoinable(
        std::bind(JetsonV4L2Capturer::CaptureThread, this), "CaptureThread",
        webrtc::ThreadAttributes().SetPriority(webrtc::ThreadPriority::kHigh));
  }

  // Needed to start UVC camera - from the uvcview application
  enum v4l2_buf_type type;
  type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (ioctl(_deviceFd, VIDIOC_STREAMON, &type) == -1) {
    RTC_LOG(LS_INFO) << "Failed to turn on stream";
    return -1;
  }

  _captureStarted = true;
  return 0;
}

int32_t JetsonV4L2Capturer::StopCapture() {
  if (!_captureThread.empty()) {
    {
      webrtc::MutexLock lock(&capture_lock_);
      quit_ = true;
    }
    _captureThread.Finalize();
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

// critical section protected by the caller

bool JetsonV4L2Capturer::AllocateVideoBuffers() {
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

  std::unique_ptr<int[]> fds;
  if (_captureVideoType != webrtc::VideoType::kMJPEG) {
    fds.reset(new int[rbuffer.count]);
    NvBufSurf::NvCommonAllocateParams params = {0};

    params.memType = NVBUF_MEM_SURFACE_ARRAY;
    params.width = _currentWidth;
    params.height = _currentHeight;
    params.layout = NVBUF_LAYOUT_PITCH;
    if (_captureVideoType == webrtc::VideoType::kYUY2)
      params.colorFormat = NVBUF_COLOR_FORMAT_YUYV;
    else if (_captureVideoType == webrtc::VideoType::kI420)
      params.colorFormat = NVBUF_COLOR_FORMAT_YUV420;
    else if (_captureVideoType == webrtc::VideoType::kYV12)
      params.colorFormat = NVBUF_COLOR_FORMAT_YVU420;
    else if (_captureVideoType == webrtc::VideoType::kUYVY)
      params.colorFormat = NVBUF_COLOR_FORMAT_UYVY;
    params.memtag = NvBufSurfaceTag_CAMERA;
    if (NvBufSurf::NvAllocate(&params, rbuffer.count, fds.get())) {
      return false;
    }
  }

  // Map the buffers
  _pool = new Buffer[rbuffer.count];

  for (unsigned int i = 0; i < rbuffer.count; i++) {
    struct v4l2_buffer buffer;
    memset(&buffer, 0, sizeof(v4l2_buffer));
    if (fds == nullptr) {
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
    } else {
      buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buffer.memory = V4L2_MEMORY_DMABUF;
      buffer.index = i;

      if (ioctl(_deviceFd, VIDIOC_QUERYBUF, &buffer) < 0) {
        return false;
      }

      _pool[i].fd = fds[i];
      _pool[i].length = buffer.length;
    }

    if (ioctl(_deviceFd, VIDIOC_QBUF, &buffer) < 0) {
      return false;
    }
  }

  if (_captureVideoType == webrtc::VideoType::kMJPEG) {
    jpeg_decoder_pool_.reset(new JetsonJpegDecoderPool());
  }

  return true;
}

bool JetsonV4L2Capturer::DeAllocateVideoBuffers() {
  if (_captureVideoType == webrtc::VideoType::kMJPEG) {
    jpeg_decoder_pool_ = nullptr;

    // unmap buffers
    for (int i = 0; i < _buffersAllocatedByDevice; i++) {
      munmap(_pool[i].start, _pool[i].length);
    }
    delete[] _pool;

  } else {
    for (int i = 0; i < _buffersAllocatedByDevice; i++) {
      NvBufSurf::NvDestroy(_pool[i].fd);
    }
    delete[] _pool;
  }

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
  {
    webrtc::MutexLock lock(&capture_lock_);

    if (quit_) {
      return false;
    } else if (retVal < 0 && errno != EINTR /* continue if interrupted */) {
      // select failed
      return false;
    } else if (retVal == 0) {
      // select timed out
      return true;
    } else if (!FD_ISSET(_deviceFd, &rSet)) {
      // not event on camera handle
      return true;
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

      OnCaptured(&buf);

      // enqueue the buffer again
      if (ioctl(_deviceFd, VIDIOC_QBUF, &buf) == -1) {
        RTC_LOG(LS_INFO) << __FUNCTION__ << " Failed to enqueue capture buffer";
      }
    }
  }
  usleep(0);
  return true;
}

void JetsonV4L2Capturer::OnCaptured(v4l2_buffer* buf) {
  const int64_t timestamp_us = webrtc::TimeMicros();
  int adapted_width, adapted_height, crop_width, crop_height, crop_x, crop_y;
  if (!AdaptFrame(_currentWidth, _currentHeight, timestamp_us, &adapted_width,
                  &adapted_height, &crop_width, &crop_height, &crop_x,
                  &crop_y)) {
    return;
  }

  if (_captureVideoType == webrtc::VideoType::kMJPEG) {
    uint8_t* data = (uint8_t*)_pool[buf->index].start;
    uint32_t bytesused = buf->bytesused;
    // 一部のカメラ (DELL WB7022) は不正なデータを送ってくることがある。
    // これをハードウェアJPEGデコーダーに送ると Momo ごとクラッシュしてしまう。
    // JPEG の先頭は SOI マーカー 0xffd8 で始まるのでチェックして落ちないようにする。
    if (bytesused < 2 || data[0] != 0xff || data[1] != 0xd8) {
      RTC_LOG(LS_WARNING) << __FUNCTION__
                          << " Invalid JPEG buffer frame skipped";
      return;
    }

    unsigned int eosSearchSize = MJPEG_EOS_SEARCH_SIZE;
    uint8_t* p;
    /* v4l2_buf.bytesused may have padding bytes for alignment
              Search for EOF to get exact size */
    if (eosSearchSize > bytesused)
      eosSearchSize = bytesused;
    for (unsigned int i = 0; i < eosSearchSize; i++) {
      p = data + bytesused;
      if ((*(p - 2) == 0xff) && (*(p - 1) == 0xd9)) {
        break;
      }
      bytesused--;
    }

    std::shared_ptr<JetsonJpegDecoder> decoder = jpeg_decoder_pool_->Pop();
    int fd = 0;
    uint32_t width, height, pixfmt;
    if (decoder->DecodeToFd(fd, data, bytesused, pixfmt, width, height) < 0) {
      RTC_LOG(LS_ERROR) << "decodeToFd Failed";
      return;
    }
    webrtc::scoped_refptr<JetsonBuffer> jetson_buffer(
        JetsonBuffer::Create(_captureVideoType, width, height, adapted_width,
                             adapted_height, fd, pixfmt, std::move(decoder)));
    OnFrame(webrtc::VideoFrame::Builder()
                .set_video_frame_buffer(jetson_buffer)
                .set_timestamp_rtp(0)
                .set_timestamp_ms(webrtc::TimeMillis())
                .set_timestamp_us(webrtc::TimeMicros())
                .set_rotation(webrtc::kVideoRotation_0)
                .build());
  } else {
    webrtc::scoped_refptr<JetsonBuffer> jetson_buffer(JetsonBuffer::Create(
        _captureVideoType, _currentWidth, _currentHeight, adapted_width,
        adapted_height, _pool[buf->index].fd, _currentPixelFormat, nullptr));
    OnFrame(webrtc::VideoFrame::Builder()
                .set_video_frame_buffer(jetson_buffer)
                .set_timestamp_rtp(0)
                .set_timestamp_ms(webrtc::TimeMillis())
                .set_timestamp_us(webrtc::TimeMicros())
                .set_rotation(webrtc::kVideoRotation_0)
                .build());
  }
}

}  // namespace sora