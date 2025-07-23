#ifndef V4L2_RUNNER_H_
#define V4L2_RUNNER_H_

#include <atomic>
#include <functional>
#include <mutex>
#include <optional>
#include <queue>
#include <string>

// Linux
#include <linux/videodev2.h>

// WebRTC
#include <rtc_base/platform_thread.h>

template <class T>
class ConcurrentQueue {
 public:
  void push(T t) {
    std::lock_guard<std::mutex> lock(mutex_);
    queue_.push(t);
  }
  std::optional<T> pop() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (queue_.empty()) {
      return std::nullopt;
    }
    T t = queue_.front();
    queue_.pop();
    return t;
  }
  bool empty() {
    std::lock_guard<std::mutex> lock(mutex_);
    return queue_.empty();
  }
  size_t size() {
    std::lock_guard<std::mutex> lock(mutex_);
    return queue_.size();
  }
  void clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    queue_ = std::queue<T>();
  }

 private:
  std::queue<T> queue_;
  std::mutex mutex_;
};

class V4L2Runner {
 public:
  ~V4L2Runner();

  static std::shared_ptr<V4L2Runner> Create(
      std::string name,
      int fd,
      int src_count,
      int src_memory,
      int dst_memory,
      std::function<void()> on_change_resolution = nullptr);

  typedef std::function<void(v4l2_buffer*, std::function<void()>)>
      OnCompleteCallback;

  int Enqueue(v4l2_buffer* v4l2_buf, OnCompleteCallback on_complete);

  std::optional<int> PopAvailableBufferIndex();

 private:
  void PollProcess();

 private:
  std::string name_;
  int fd_;
  int src_count_;
  int src_memory_;
  int dst_memory_;
  std::function<void()> on_change_resolution_;

  ConcurrentQueue<int> output_buffers_available_;
  ConcurrentQueue<OnCompleteCallback> on_completes_;
  std::atomic<bool> abort_poll_;
  webrtc::PlatformThread thread_;
};

#endif
