#ifndef ROS_VIDEO_CAPTURE_H_
#define ROS_VIDEO_CAPTURE_H_

#include "ros/ros.h"

#include "sensor_msgs/Image.h"
#include "sensor_msgs/CompressedImage.h"
#include "media/base/videocapturer.h"

#include <mutex>

class ROSVideoCapture : public cricket::VideoCapturer
{
public:
  explicit ROSVideoCapture();
  ~ROSVideoCapture() override;

  // cricket::VideoCapturer interface.
  cricket::CaptureState Start(
      const cricket::VideoFormat &capture_format) override;
  void Stop() override;

  bool IsRunning() override;
  bool GetPreferredFourccs(std::vector<uint32_t> *fourccs) override;

  bool GetBestCaptureFormat(const cricket::VideoFormat &desired,
                            cricket::VideoFormat *best_format) override;
  bool IsScreencast() const override;

  void ROSCallbackRaw(const sensor_msgs::ImageConstPtr &image);
  void ROSCallbackCompressed(const sensor_msgs::CompressedImageConstPtr &image);

private:
  static uint32_t ConvertEncodingType(const std::string encoding);
  void ROSCallback(ros::Time ros_time, const uint8_t* sample, size_t sample_size, int src_width, int src_height, uint32_t fourcc);

  std::mutex mtx_;
  bool running_;
  ros::Subscriber sub_; 
};

#endif