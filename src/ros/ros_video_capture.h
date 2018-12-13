#ifndef ROS_VIDEO_CAPTURE_H_
#define ROS_VIDEO_CAPTURE_H_

#include "ros/ros.h"

#include "sensor_msgs/Image.h"
#include "sensor_msgs/CompressedImage.h"
#include "media/base/videocapturer.h"

#include <thread>
#include <mutex>
#include <condition_variable>

#include "connection_settings.h"
#include "signal_listener.h"

class ROSVideoCapture : public cricket::VideoCapturer, public SignalListener
{
public:
  explicit ROSVideoCapture(ConnectionSettings conn_settings);
  ~ROSVideoCapture() override;

  bool Init();

  void OnSignal(int signum) override;

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

  ros::AsyncSpinner* spinner_;
  ros::Subscriber sub_; 
  std::mutex mtx_;
  std::condition_variable condition_;
  bool running_;
  uint64_t last_time_ns_;
  int width_;
  int height_;
  int64_t interval_; 
  bool signal_received_ = false;
};

#endif