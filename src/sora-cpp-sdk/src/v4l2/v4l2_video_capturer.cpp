/*
 *  Copyright (c) 2012 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "sora/v4l2/v4l2_video_capturer.h"

#include <stdio.h>
#include <string.h>
#include <time.h>
#include <algorithm>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

// Linux
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/select.h>
#include <unistd.h>

// WebRTC
#include <api/make_ref_counted.h>
#include <api/scoped_refptr.h>
#include <api/video/i420_buffer.h>
#include <api/video/nv12_buffer.h>
#include <api/video/video_frame.h>
#include <api/video/video_frame_buffer.h>
#include <api/video/video_rotation.h>
#include <bits/types/struct_timeval.h>
#include <common_video/libyuv/include/webrtc_libyuv.h>
#include <media/base/video_common.h>
#include <modules/video_capture/video_capture.h>
#include <modules/video_capture/video_capture_factory.h>
#include <rtc_base/logging.h>
#include <rtc_base/platform_thread.h>
#include <rtc_base/synchronization/mutex.h>
#include <rtc_base/time_utils.h>

// libyuv
#include <libyuv/convert.h>
#include <libyuv/rotate.h>

#include "sora/scalable_track_source.h"
#include "sora/v4l2/v4l2_device.h"

#define MJPEG_EOS_SEARCH_SIZE 4096

namespace sora {

webrtc::scoped_refptr<V4L2VideoCapturer> V4L2VideoCapturer::Create(
    const V4L2VideoCapturerConfig& config) {
  auto capturer = webrtc::make_ref_counted<V4L2VideoCapturer>(config);
  if (capturer->Init() != 0) {
    RTC_LOG(LS_ERROR) << "Failed to initialize V4L2VideoCapturer";
    return nullptr;
  }
  if (capturer->StartCapture() != 0) {
    RTC_LOG(LS_ERROR) << "Failed to start capture";
    return nullptr;
  }
  return capturer;
}

V4L2VideoCapturer::V4L2VideoCapturer(const V4L2VideoCapturerConfig& config)
    : ScalableVideoTrackSource(config),
      config_(config),
      _deviceFd(-1),
      _buffersAllocatedByDevice(-1),
      _currentWidth(-1),
      _currentHeight(-1),
      _currentFrameRate(-1),
      _useNative(false),
      _captureStarted(false),
      _captureVideoType(webrtc::VideoType::kI420),
      _pool(NULL) {}

int32_t V4L2VideoCapturer::Init() {
  auto devices = EnumV4L2CaptureDevices();
  if (!devices) {
    RTC_LOG(LS_ERROR) << "Failed to enumerate V4L2 devices";
    return -1;
  }
  if (devices->size() == 0) {
    RTC_LOG(LS_ERROR) << "No V4L2 capture devices found";
    return -1;
  }
  if (config_.video_device.empty()) {
    // デバイスが指定されていない場合は最初のデバイスを使う
    device_ = devices->at(0);
    RTC_LOG(LS_INFO) << "Using first video device: " << device_.path << " ("
                     << device_.card << ", " << device_.bus_info << ")";
  } else {
    // 指定されたデバイス名に一致するデバイスを探す
    bool found = false;
    for (const auto& device : *devices) {
      if (config_.video_device == device.card ||
          config_.video_device == device.path) {
        device_ = device;
        found = true;
        break;
      }
    }
    if (!found) {
      RTC_LOG(LS_ERROR) << "Specified video device not found: "
                        << config_.video_device;
      return -1;
    }
  }

  return 0;
}

V4L2VideoCapturer::~V4L2VideoCapturer() {
  StopCapture();
  if (_deviceFd != -1)
    close(_deviceFd);
}

int32_t V4L2VideoCapturer::StartCapture() {
  if (_captureStarted) {
    return 0;
  }

  webrtc::MutexLock lock(&capture_lock_);
  // first open /dev/video device
  if ((_deviceFd = open(device_.path.c_str(), O_RDWR | O_NONBLOCK, 0)) < 0) {
    RTC_LOG(LS_INFO) << "error in opening " << device_.path
                     << " errono = " << errno;
    return -1;
  }

  // 順番にサポートしているビデオフォーマットを探す。
  // 特に指定が無ければ VGA より大きいのは MJPEG 優先、それ以下は I420 優先となる。
  int nFormats = 0;
  const int MaxFormats = 6;
  unsigned int fmts[MaxFormats] = {};
  if (config_.force_yuy2) {
    fmts[0] = V4L2_PIX_FMT_YUYV;
    nFormats = 1;
  } else if (config_.force_i420) {
    fmts[0] = V4L2_PIX_FMT_YUV420;
    nFormats = 1;
  } else if (config_.force_nv12) {
    fmts[0] = V4L2_PIX_FMT_NV12;
    nFormats = 1;
  } else if (config_.use_native) {
    fmts[0] = V4L2_PIX_FMT_MJPEG;
    fmts[1] = V4L2_PIX_FMT_JPEG;
    nFormats = 2;
  } else if (config_.width > 640 || config_.height > 480) {
    fmts[0] = V4L2_PIX_FMT_MJPEG;
    fmts[1] = V4L2_PIX_FMT_YUV420;
    fmts[2] = V4L2_PIX_FMT_YVU420;
    fmts[3] = V4L2_PIX_FMT_YUYV;
    fmts[4] = V4L2_PIX_FMT_UYVY;
    fmts[5] = V4L2_PIX_FMT_JPEG;
    nFormats = 6;
  } else {
    fmts[0] = V4L2_PIX_FMT_YUV420;
    fmts[1] = V4L2_PIX_FMT_YVU420;
    fmts[2] = V4L2_PIX_FMT_YUYV;
    fmts[3] = V4L2_PIX_FMT_UYVY;
    fmts[4] = V4L2_PIX_FMT_MJPEG;
    fmts[5] = V4L2_PIX_FMT_JPEG;
    nFormats = 6;
  }

  std::optional<uint32_t> found_format;
  for (int i = 0; i < nFormats; i++) {
    for (int j = 0; j < device_.format_descriptions.size(); j++) {
      if (fmts[i] == device_.format_descriptions[j].pixel_format) {
        found_format = fmts[i];
        break;
      }
    }
  }
  if (!found_format) {
    RTC_LOG(LS_ERROR) << "No supporting video formats found";
    return -1;
  }

  // ビデオフォーマットとサイズを設定する
  struct v4l2_format video_fmt;
  memset(&video_fmt, 0, sizeof(struct v4l2_format));
  video_fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  video_fmt.fmt.pix.sizeimage = 0;
  video_fmt.fmt.pix.width = config_.width;
  video_fmt.fmt.pix.height = config_.height;
  video_fmt.fmt.pix.pixelformat = *found_format;

  if (ioctl(_deviceFd, VIDIOC_S_FMT, &video_fmt) < 0) {
    RTC_LOG(LS_INFO) << "error in VIDIOC_S_FMT, errno = " << errno;
    return -1;
  }

  // FOURCC を webrtc::VideoType に変換する
  if (video_fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_YUYV)
    _captureVideoType = webrtc::VideoType::kYUY2;
  else if (video_fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_YUV420)
    _captureVideoType = webrtc::VideoType::kI420;
  else if (video_fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_YVU420)
    _captureVideoType = webrtc::VideoType::kYV12;
  else if (video_fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_UYVY)
    _captureVideoType = webrtc::VideoType::kUYVY;
  else if (video_fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_NV12)
    _captureVideoType = webrtc::VideoType::kNV12;
  else if (video_fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_MJPEG ||
           video_fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_JPEG)
    _captureVideoType = webrtc::VideoType::kMJPEG;

  // 現在の幅と高さを保存しておく
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
      streamparms.parm.capture.timeperframe.denominator = config_.framerate;
      if (ioctl(_deviceFd, VIDIOC_S_PARM, &streamparms) < 0) {
        RTC_LOG(LS_INFO) << "Failed to set the framerate. errno=" << errno;
        driver_framerate_support = false;
      } else {
        _currentFrameRate = config_.framerate;
      }
    }
  }
  // If driver doesn't support framerate control, need to hardcode.
  // Hardcoding the value based on the frame size.
  if (!driver_framerate_support) {
    if (!config_.use_native && _currentWidth >= 800 &&
        _captureVideoType != webrtc::VideoType::kMJPEG) {
      _currentFrameRate = 15;
    } else {
      _currentFrameRate = 30;
    }
  }

  RTC_LOG(LS_INFO) << "Camera is started with format: "
                   << webrtc::GetFourccName(video_fmt.fmt.pix.pixelformat)
                   << " size: " << _currentWidth << "x" << _currentHeight
                   << " fps: " << _currentFrameRate;

  if (!AllocateVideoBuffers()) {
    RTC_LOG(LS_INFO) << "failed to allocate video capture buffers";
    return -1;
  }

  // start capture thread;
  if (_captureThread.empty()) {
    quit_ = false;
    _captureThread = webrtc::PlatformThread::SpawnJoinable(
        std::bind(V4L2VideoCapturer::CaptureThread, this), "CaptureThread",
        webrtc::ThreadAttributes().SetPriority(webrtc::ThreadPriority::kHigh));
  }

  // Needed to start UVC camera - from the uvcview application
  enum v4l2_buf_type type;
  type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (ioctl(_deviceFd, VIDIOC_STREAMON, &type) == -1) {
    RTC_LOG(LS_INFO) << "Failed to turn on stream";
    return -1;
  }

  _useNative = config_.use_native;
  _captureStarted = true;
  return 0;
}

int32_t V4L2VideoCapturer::StopCapture() {
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
      buf.memory = V4L2_MEMORY_MMAP;
      // dequeue a buffer - repeat until dequeued properly!
      while (ioctl(_deviceFd, VIDIOC_DQBUF, &buf) < 0) {
        if (errno != EINTR) {
          RTC_LOG(LS_INFO) << "could not sync on a buffer on device "
                           << strerror(errno);
          return true;
        }
      }

      uint8_t* data = (uint8_t*)_pool[buf.index].start;
      uint32_t bytesused = buf.bytesused;
      // 一部のカメラ (DELL WB7022) は不正なデータを送ってくることがある。
      // これをハードウェアJPEGデコーダーに送ると Momo ごとクラッシュしてしまう。
      // JPEG の先頭は SOI マーカー 0xffd8 で始まるのでチェックして落ちないようにする。
      if (_captureVideoType == webrtc::VideoType::kMJPEG && bytesused >= 2) {
        if (data[0] != 0xff || data[1] != 0xd8) {
          RTC_LOG(LS_WARNING)
              << __FUNCTION__ << " Invalid JPEG buffer frame skipped";
        } else {
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
          OnCaptured(data, bytesused);
        }
      } else {
        OnCaptured(data, bytesused);
      }

      // enqueue the buffer again
      if (ioctl(_deviceFd, VIDIOC_QBUF, &buf) == -1) {
        RTC_LOG(LS_INFO) << __FUNCTION__ << " Failed to enqueue capture buffer";
      }
    }
  }
  usleep(0);
  return true;
}

void V4L2VideoCapturer::OnCaptured(uint8_t* data, uint32_t bytesused) {
  webrtc::scoped_refptr<webrtc::VideoFrameBuffer> dst_buffer = nullptr;

  if (_captureVideoType == webrtc::VideoType::kNV12) {
    // NV12 の場合はそのまま使用
    webrtc::scoped_refptr<webrtc::NV12Buffer> nv12_buffer =
        webrtc::NV12Buffer::Create(_currentWidth, _currentHeight);
    nv12_buffer->InitializeData();
    const uint8_t* src_y = data;
    const uint8_t* src_uv = data + _currentWidth * _currentHeight;
    if (libyuv::NV12Copy(src_y, _currentWidth, src_uv, _currentWidth,
                         nv12_buffer->MutableDataY(), nv12_buffer->StrideY(),
                         nv12_buffer->MutableDataUV(), nv12_buffer->StrideUV(),
                         _currentWidth, _currentHeight) < 0) {
      RTC_LOG(LS_ERROR) << "NV12Copy Failed";
    } else {
      dst_buffer = nv12_buffer;
    }
  } else if (_captureVideoType == webrtc::VideoType::kYUY2) {
    // YUY2 の場合は NV12 に変換
    webrtc::scoped_refptr<webrtc::NV12Buffer> nv12_buffer =
        webrtc::NV12Buffer::Create(_currentWidth, _currentHeight);
    nv12_buffer->InitializeData();
    if (libyuv::YUY2ToNV12(data, _currentWidth * 2, nv12_buffer->MutableDataY(),
                           nv12_buffer->StrideY(), nv12_buffer->MutableDataUV(),
                           nv12_buffer->StrideUV(), _currentWidth,
                           _currentHeight) < 0) {
      RTC_LOG(LS_ERROR) << "YUY2ToNV12 Failed";
    } else {
      dst_buffer = nv12_buffer;
    }
  } else {
    // それ以外の場合は I420 に変換
    webrtc::scoped_refptr<webrtc::I420Buffer> i420_buffer(
        webrtc::I420Buffer::Create(_currentWidth, _currentHeight));
    i420_buffer->InitializeData();
    if (libyuv::ConvertToI420(
            data, bytesused, i420_buffer.get()->MutableDataY(),
            i420_buffer.get()->StrideY(), i420_buffer.get()->MutableDataU(),
            i420_buffer.get()->StrideU(), i420_buffer.get()->MutableDataV(),
            i420_buffer.get()->StrideV(), 0, 0, _currentWidth, _currentHeight,
            _currentWidth, _currentHeight, libyuv::kRotate0,
            ConvertVideoType(_captureVideoType)) < 0) {
      RTC_LOG(LS_ERROR) << "ConvertToI420 Failed";
    } else {
      dst_buffer = i420_buffer;
    }
  }

  if (dst_buffer) {
    webrtc::VideoFrame video_frame = webrtc::VideoFrame::Builder()
                                         .set_video_frame_buffer(dst_buffer)
                                         .set_timestamp_rtp(0)
                                         .set_timestamp_ms(webrtc::TimeMillis())
                                         .set_timestamp_us(webrtc::TimeMicros())
                                         .set_rotation(webrtc::kVideoRotation_0)
                                         .build();
    OnCapturedFrame(video_frame);
  }
}

}  // namespace sora