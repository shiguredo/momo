#ifndef ROS_VIDEO_CAPTURE_H_
#define ROS_VIDEO_CAPTURE_H_

// ROS
#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>

#include "connection_settings.h"
#include "rtc/scalable_track_source.h"

class ROSVideoCapture : public ScalableVideoTrackSource {
 public:
  explicit ROSVideoCapture(ConnectionSettings cs);
  ~ROSVideoCapture();

  void Destroy();

  // ROS Callback
  void ROSCallbackRaw(const sensor_msgs::ImageConstPtr& image);
  void ROSCallbackCompressed(const sensor_msgs::CompressedImageConstPtr& image);

 private:
  static uint32_t ConvertEncodingType(const std::string encoding);
  void ROSCallback(ros::Time ros_time,
                   const uint8_t* sample,
                   size_t sample_size,
                   int src_width,
                   int src_height,
                   uint32_t fourcc);

  ros::AsyncSpinner* spinner_;
  ros::Subscriber sub_;
};

#endif
