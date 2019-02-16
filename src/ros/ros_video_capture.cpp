#include "ros_video_capture.h"

#include <string.h>
#include <unistd.h>

#include "api/video/i420_buffer.h"
#include "rtc_base/logsinks.h"
#include "third_party/libyuv/include/libyuv.h"
#include "sensor_msgs/image_encodings.h"

ROSVideoCapture::ROSVideoCapture(ConnectionSettings cs)
{
  ros::NodeHandle nh;
  if (cs.image_compressed)
  {
    sub_ = nh.subscribe<sensor_msgs::CompressedImage>(cs.camera_name, 1, boost::bind(&ROSVideoCapture::ROSCallbackCompressed, this, _1));
  }
  else
  {
    sub_ = nh.subscribe<sensor_msgs::Image>(cs.camera_name, 1, boost::bind(&ROSVideoCapture::ROSCallbackRaw, this, _1));
  }

  spinner_ = new ros::AsyncSpinner(1);
  spinner_->start();
}

ROSVideoCapture::~ROSVideoCapture()
{
  Destroy();
}

void ROSVideoCapture::Destroy()
{
  spinner_->stop();
}

void ROSVideoCapture::OnFrame(const webrtc::VideoFrame& frame) {
  VideoCapturer::OnFrame(frame);
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

  webrtc::VideoFrame captureFrame =
          webrtc::VideoFrame::Builder()
              .set_video_frame_buffer(dst_buffer)
              .set_rotation(webrtc::kVideoRotation_0)
              .set_timestamp_us((int64_t)(ros_time.toNSec() / 1000))
              .build();
  OnFrame(captureFrame);
}