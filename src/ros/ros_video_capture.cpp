#include "ros_video_capture.h"

#include <string.h>
#include "api/video/i420_buffer.h"
#include "rtc_base/logsinks.h"
#include "third_party/libyuv/include/libyuv.h"
#include "sensor_msgs/image_encodings.h"

ROSVideoCapture::ROSVideoCapture(ConnectionSettings conn_settings) : running_(false)
{
  ros::NodeHandle nh;
  if (conn_settings.image_compressed) {
    sub_ = nh.subscribe<sensor_msgs::CompressedImage>(conn_settings.camera_name, 1, boost::bind(&ROSVideoCapture::ROSCallbackCompressed, this, _1));
  } else {
    sub_ = nh.subscribe<sensor_msgs::Image>(conn_settings.camera_name, 1, boost::bind(&ROSVideoCapture::ROSCallbackRaw, this, _1));
  }

  std::vector<cricket::VideoFormat> formats;
  formats.push_back(cricket::VideoFormat(640, 480,
                                         cricket::VideoFormat::FpsToInterval(30), cricket::FOURCC_I420));
  SetSupportedFormats(formats);

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
  int64_t ntp_time_ms = ros_time.sec * 1000;
  ntp_time_ms += ros_time.nsec / 1000000;
  if (ntp_time_ms % 1000000 >= 500000)
  {
    ntp_time_ms++;
  }
  captureFrame.set_ntp_time_ms(ntp_time_ms);
  std::unique_lock<std::mutex> lk(mtx_);
  if(!running_) return;
  OnFrame(captureFrame, src_width, src_height);
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
  best_format->width = 640;
  best_format->height = 480;
  best_format->fourcc = cricket::FOURCC_I420;
  best_format->interval = desired.interval;
  return true;
}

bool ROSVideoCapture::IsScreencast() const
{
  return false;
}