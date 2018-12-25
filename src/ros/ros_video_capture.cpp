#include "ros_video_capture.h"

#include <string.h>
#include <unistd.h>

#include "api/video/i420_buffer.h"
#include "rtc_base/logsinks.h"
#include "third_party/libyuv/include/libyuv.h"
#include "sensor_msgs/image_encodings.h"

ROSVideoCapture::ROSVideoCapture(ConnectionSettings conn_settings)
    : running_(false), last_time_ns_(0), width_(0), height_(0), interval_(0)
{
  ros::NodeHandle nh;
  if (conn_settings.image_compressed)
  {
    sub_ = nh.subscribe<sensor_msgs::CompressedImage>(conn_settings.camera_name, 1, boost::bind(&ROSVideoCapture::ROSCallbackCompressed, this, _1));
  }
  else
  {
    sub_ = nh.subscribe<sensor_msgs::Image>(conn_settings.camera_name, 1, boost::bind(&ROSVideoCapture::ROSCallbackRaw, this, _1));
  }

  spinner_ = new ros::AsyncSpinner(1);
  spinner_->start();
}

ROSVideoCapture::~ROSVideoCapture()
{
  spinner_->stop();
}

uint32_t ROSVideoCapture::ConvertEncodingType(const std::string encoding)
{
  if (encoding == sensor_msgs::image_encodings::RGB8)
  {
    return libyuv::FOURCC_RAW;
  }
  else if (encoding == sensor_msgs::image_encodings::BGR8)
  {
    return libyuv::FOURCC_24BG;
  }
  else if (encoding == sensor_msgs::image_encodings::RGBA8)
  {
    return libyuv::FOURCC_BGRA;
  }
  else if (encoding == sensor_msgs::image_encodings::BGRA8)
  {
    return libyuv::FOURCC_RGBA;
  }
  return libyuv::FOURCC_ANY;
}

void ROSVideoCapture::ROSCallbackRaw(const sensor_msgs::ImageConstPtr &image)
{
  ROSCallback(image->header.stamp, image->data.data(), image->data.size(), image->width, image->height, ConvertEncodingType(image->encoding));
}

void ROSVideoCapture::ROSCallbackCompressed(const sensor_msgs::CompressedImageConstPtr &image)
{
  int width, height;
  if (libyuv::MJPGSize(image->data.data(), image->data.size(), &width, &height) < 0)
  {
    RTC_LOG(LS_ERROR) << "MJPGSize Failed";
    return;
  }
  ROSCallback(image->header.stamp, image->data.data(), image->data.size(), width, height, libyuv::FOURCC_MJPG);
}

void ROSVideoCapture::ROSCallback(ros::Time ros_time, const uint8_t *sample, size_t sample_size, int src_width, int src_height, uint32_t fourcc)
{
  rtc::scoped_refptr<webrtc::I420Buffer> dst_buffer(webrtc::I420Buffer::Create(src_width, src_height));
  dst_buffer->InitializeData();

  if (libyuv::ConvertToI420(sample, sample_size,
                            dst_buffer.get()->MutableDataY(), dst_buffer.get()->StrideY(),
                            dst_buffer.get()->MutableDataU(), dst_buffer.get()->StrideU(),
                            dst_buffer.get()->MutableDataV(), dst_buffer.get()->StrideV(),
                            0, 0, src_width, src_height, src_width, src_height,
                            libyuv::kRotate0, fourcc) < 0)
  {
    RTC_LOG(LS_ERROR) << "ConvertToI420 Failed";
    return;
  }

  webrtc::VideoFrame captureFrame(dst_buffer, 0, rtc::TimeMillis(), webrtc::kVideoRotation_0);
  uint64_t ros_time_ns = ros_time.toNSec();
  if (last_time_ns_ != 0) {
    interval_ = ros_time_ns - last_time_ns_;
  }
  last_time_ns_ = ros_time_ns;
  width_ = src_width;
  height_ = src_height;
  captureFrame.set_ntp_time_ms((int64_t)(ros_time_ns / 1000000));
  std::unique_lock<std::mutex> lk(mtx_);
  if (!running_) {
    if (interval_ != 0) {
      condition_.notify_all();
    }
    return;
  }
  OnFrame(captureFrame, src_width, src_height);
}

bool ROSVideoCapture::Init()
{
  std::unique_lock<std::mutex> lk(mtx_);
  // 確実にラムダ式の条件が満たされるまで待つ（既に満たされていたらそもそも wait しない）
  condition_.wait(lk, [this]() { return signal_received_ || interval_ != 0; });
  if (signal_received_) {
    return false;
  }
  // 必ず interval_ != 0 になっているので処理を続行

  std::vector<cricket::VideoFormat> formats;
  formats.push_back(cricket::VideoFormat(width_, height_, interval_, cricket::FOURCC_I420));
  SetSupportedFormats(formats);
  return true;
}

void ROSVideoCapture::OnSignal(int signum)
{
  std::unique_lock<std::mutex> lk(mtx_);
  // 単純に notify_all するだけだと spurious wakeup との区別が付かないのでメンバ変数を用意する
  signal_received_ = true;
  condition_.notify_all();
}

cricket::CaptureState ROSVideoCapture::Start(
    const cricket::VideoFormat &capture_format)
{
  std::unique_lock<std::mutex> lk(mtx_);
  running_ = true;
  SetCaptureState(cricket::CS_RUNNING);
  return cricket::CS_RUNNING;
}

void ROSVideoCapture::Stop()
{
  std::unique_lock<std::mutex> lk(mtx_);
  running_ = false;
  SetCaptureState(cricket::CS_STOPPED);
}

bool ROSVideoCapture::IsRunning()
{
  return running_;
}

bool ROSVideoCapture::GetPreferredFourccs(
    std::vector<uint32_t> *fourccs)
{
  if (!fourccs)
    return false;
  fourccs->push_back(cricket::FOURCC_I420);
  return true;
}

bool ROSVideoCapture::GetBestCaptureFormat(const cricket::VideoFormat &desired,
                                           cricket::VideoFormat *best_format)
{
  if (!best_format)
    return false;
  best_format->width = width_;
  best_format->height = height_;
  best_format->fourcc = cricket::FOURCC_I420;
  best_format->interval = desired.interval;
  return true;
}

bool ROSVideoCapture::IsScreencast() const
{
  return false;
}