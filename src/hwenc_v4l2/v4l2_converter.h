#ifndef V4L2_CONVERTER_H_
#define V4L2_CONVERTER_H_

#include <functional>
#include <memory>

// Linux
#include <linux/videodev2.h>

// WebRTC
#include <api/scoped_refptr.h>
#include <api/video/video_frame_buffer.h>

#include "v4l2_buffers.h"
#include "v4l2_runner.h"

class V4L2Helper {
 public:
  static void InitFormat(int type,
                         int width,
                         int height,
                         int pixelformat,
                         int bytesperline,
                         int sizeimage,
                         v4l2_format* fmt);
  static int QueueBuffers(int fd, const V4L2Buffers& buffers);
};

class V4L2H264EncodeConverter {
 public:
  typedef std::function<void(uint8_t*, int, int64_t, bool)> OnCompleteCallback;

  static std::shared_ptr<V4L2H264EncodeConverter> Create(int src_memory,
                                                         int src_width,
                                                         int src_height,
                                                         int src_stride);

 private:
  static constexpr int NUM_OUTPUT_BUFFERS = 4;
  static constexpr int NUM_CAPTURE_BUFFERS = 4;

  int Init(int src_memory, int src_width, int src_height, int src_stride);

 public:
  int fd() const;

  int Encode(const rtc::scoped_refptr<webrtc::VideoFrameBuffer>& frame_buffer,
             int64_t timestamp_us,
             bool force_key_frame,
             OnCompleteCallback on_complete);

  ~V4L2H264EncodeConverter();

 private:
  int fd_ = 0;

  V4L2Buffers src_buffers_;
  V4L2Buffers dst_buffers_;

  std::shared_ptr<V4L2Runner> runner_;
};

class V4L2ScaleConverter {
 public:
  typedef std::function<void(rtc::scoped_refptr<webrtc::VideoFrameBuffer>,
                             int64_t)>
      OnCompleteCallback;

  static std::shared_ptr<V4L2ScaleConverter> Create(int src_memory,
                                                    int src_width,
                                                    int src_height,
                                                    int src_stride,
                                                    bool dst_export_dmafds,
                                                    int dst_width,
                                                    int dst_height,
                                                    int dst_stride);

 private:
  static constexpr int NUM_OUTPUT_BUFFERS = 4;
  static constexpr int NUM_CAPTURE_BUFFERS = 4;

  int Init(int src_memory,
           int src_width,
           int src_height,
           int src_stride,
           bool dst_export_dmafds,
           int dst_width,
           int dst_height,
           int dst_stride);

 public:
  int Scale(const rtc::scoped_refptr<webrtc::VideoFrameBuffer>& frame_buffer,
            int64_t timestamp_us,
            OnCompleteCallback on_complete);

  ~V4L2ScaleConverter();

 private:
  int fd_ = 0;

  int dst_width_ = 0;
  int dst_height_ = 0;
  int dst_stride_ = 0;

  V4L2Buffers src_buffers_;
  V4L2Buffers dst_buffers_;

  std::shared_ptr<V4L2Runner> runner_;
};

class V4L2DecodeConverter {
 public:
  typedef std::function<void(rtc::scoped_refptr<webrtc::VideoFrameBuffer>,
                             int64_t)>
      OnCompleteCallback;

  static std::shared_ptr<V4L2DecodeConverter> Create(int src_pixelformat,
                                                     bool dst_export_dmafds);

  // デコード後のサイズが分かる（かつ V4L2_EVENT_SRC_CH_RESOLUTION イベントが飛んでこないフォーマットの）場合はこっちを使う
  static std::shared_ptr<V4L2DecodeConverter> Create(int src_pixelformat,
                                                     bool dst_export_dmafds,
                                                     int dst_width,
                                                     int dst_height,
                                                     int dst_stride);

 private:
  static constexpr int NUM_OUTPUT_BUFFERS = 4;
  static constexpr int NUM_CAPTURE_BUFFERS = 4;

  int Init(int src_pixelformat, bool dst_export_dmafds);

  int Init(int src_pixelformat,
           bool dst_export_dmafds,
           int dst_width,
           int dst_height,
           int dst_stride);

 public:
  int fd() const;

  int Decode(const uint8_t* data,
             int size,
             int64_t timestamp_rtp,
             OnCompleteCallback on_complete);

  ~V4L2DecodeConverter();

 private:
  int fd_ = 0;

  int dst_width_ = 0;
  int dst_height_ = 0;
  int dst_stride_ = 0;

  V4L2Buffers src_buffers_;
  V4L2Buffers dst_buffers_;

  std::shared_ptr<V4L2Runner> runner_;
};

#endif
