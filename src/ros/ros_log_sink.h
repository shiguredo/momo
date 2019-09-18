#ifndef ROS_LOG_SINK_H_
#define ROS_LOG_SINK_H_

#include <ros/ros.h>

#include "rtc_base/log_sinks.h"
#include "rtc_base/logging.h"

class ROSLogSink : public rtc::LogSink {
 public:
  void OnLogMessage(const std::string& message) override {
    ROS_INFO_STREAM(message);
  }

  void OnLogMessage(const std::string& message,
                    rtc::LoggingSeverity severity,
                    const char* tag) override {
    switch (severity) {
      case rtc::LS_VERBOSE:
      case rtc::LS_NONE:
        ROS_DEBUG_STREAM(message);
        break;
      case rtc::LS_INFO:
        ROS_INFO_STREAM(message);
        break;
      case rtc::LS_WARNING:
        ROS_WARN_STREAM(message);
        break;
      case rtc::LS_ERROR:
        ROS_ERROR_STREAM(message);
        break;
    }
  }
};

#endif