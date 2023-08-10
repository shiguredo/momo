#include "v4l2_runner.h"

// Linux
#include <poll.h>
#include <sys/ioctl.h>

// WebRTC
#include <modules/video_coding/include/video_error_codes.h>
#include <rtc_base/logging.h>

V4L2Runner::~V4L2Runner() {
  abort_poll_ = true;
  thread_.Finalize();
}

std::shared_ptr<V4L2Runner> V4L2Runner::Create(
    std::string name,
    int fd,
    int src_count,
    int src_memory,
    int dst_memory,
    std::function<void()> on_change_resolution) {
  auto p = std::make_shared<V4L2Runner>();
  p->name_ = name;
  p->fd_ = fd;
  p->src_count_ = src_count;
  p->src_memory_ = src_memory;
  p->dst_memory_ = dst_memory;

  if (on_change_resolution) {
    v4l2_event_subscription sub = {};
    sub.type = V4L2_EVENT_SOURCE_CHANGE;
    if (ioctl(fd, VIDIOC_SUBSCRIBE_EVENT, &sub) < 0) {
      RTC_LOG(LS_ERROR) << "Failed to subscribe to V4L2_EVENT_SOURCE_CHANGE";
      return nullptr;
    }
    p->on_change_resolution_ = on_change_resolution;
  }

  p->abort_poll_ = false;
  p->thread_ = rtc::PlatformThread::SpawnJoinable(
      [p = p.get()]() { p->PollProcess(); }, "PollThread",
      rtc::ThreadAttributes().SetPriority(rtc::ThreadPriority::kHigh));
  for (int i = 0; i < src_count; i++) {
    p->output_buffers_available_.push(i);
  }
  return p;
}

int V4L2Runner::Enqueue(v4l2_buffer* v4l2_buf, OnCompleteCallback on_complete) {
  if (ioctl(fd_, VIDIOC_QBUF, v4l2_buf) < 0) {
    RTC_LOG(LS_ERROR) << __FUNCTION__
                      << "  Failed to queue output buffer: error="
                      << strerror(errno);
    return WEBRTC_VIDEO_CODEC_ERROR;
  }
  on_completes_.push(on_complete);

  return WEBRTC_VIDEO_CODEC_OK;
}

std::optional<int> V4L2Runner::PopAvailableBufferIndex() {
  return output_buffers_available_.pop();
}

void V4L2Runner::PollProcess() {
  while (true) {
    RTC_LOG(LS_VERBOSE) << "[POLL][" << name_ << "] Start poll";
    pollfd p = {fd_, POLLIN | POLLPRI, 0};
    int ret = poll(&p, 1, 500);
    if (abort_poll_ && output_buffers_available_.size() == src_count_) {
      break;
    }
    if (ret == -1) {
      RTC_LOG(LS_ERROR) << "[POLL][" << name_
                        << "] unexpected error ret=" << ret;
      break;
    }
    if (p.revents & POLLPRI) {
      RTC_LOG(LS_VERBOSE) << "[POLL][" << name_ << "] Polled POLLPRI";
      if (on_change_resolution_) {
        v4l2_event event = {};
        if (ioctl(fd_, VIDIOC_DQEVENT, &event) < 0) {
          RTC_LOG(LS_ERROR)
              << "[POLL][" << name_ << "] Failed dequeing an event";
          break;
        }
        if (event.type == V4L2_EVENT_SOURCE_CHANGE &&
            (event.u.src_change.changes & V4L2_EVENT_SRC_CH_RESOLUTION) != 0) {
          RTC_LOG(LS_VERBOSE) << "On change resolution";
          on_change_resolution_();
        }
      }
    }
    if (p.revents & POLLIN) {
      RTC_LOG(LS_VERBOSE) << "[POLL][" << name_ << "] Polled POLLIN";
      v4l2_buffer v4l2_buf = {};
      v4l2_buf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
      v4l2_buf.memory = src_memory_;
      v4l2_buf.length = 1;
      v4l2_plane planes[VIDEO_MAX_PLANES] = {};
      v4l2_buf.m.planes = planes;
      RTC_LOG(LS_VERBOSE) << "[POLL][" << name_ << "] DQBUF output";
      int ret = ioctl(fd_, VIDIOC_DQBUF, &v4l2_buf);
      if (ret != 0) {
        RTC_LOG(LS_ERROR) << "[POLL][" << name_
                          << "] Failed to dequeue output buffer: error="
                          << strerror(errno);
      } else {
        output_buffers_available_.push(v4l2_buf.index);
      }

      v4l2_buf = {};
      memset(planes, 0, sizeof(planes));
      v4l2_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
      v4l2_buf.memory = V4L2_MEMORY_MMAP;
      v4l2_buf.length = 1;
      v4l2_buf.m.planes = planes;
      RTC_LOG(LS_VERBOSE) << "[POLL][" << name_ << "] DQBUF capture";
      ret = ioctl(fd_, VIDIOC_DQBUF, &v4l2_buf);
      if (ret != 0) {
        RTC_LOG(LS_ERROR) << "[POLL][" << name_
                          << "] Failed to dequeue capture buffer: error="
                          << strerror(errno);
      } else {
        if (abort_poll_) {
          break;
        }
        std::optional<OnCompleteCallback> on_complete = on_completes_.pop();
        if (!on_complete) {
          RTC_LOG(LS_ERROR)
              << "[POLL][" << name_ << "] on_completes_ is empty.";
        } else {
          (*on_complete)(&v4l2_buf, [fd = fd_, v4l2_buf]() mutable {
            v4l2_plane planes[VIDEO_MAX_PLANES] = {};
            v4l2_buf.m.planes = planes;
            if (ioctl(fd, VIDIOC_QBUF, &v4l2_buf) < 0) {
              RTC_LOG(LS_ERROR) << "Failed to enqueue capture buffer: error="
                                << strerror(errno);
            }
          });
        }
      }
      RTC_LOG(LS_VERBOSE) << "[POLL][" << name_ << "] Completed POLLIN";
    }
  }
}