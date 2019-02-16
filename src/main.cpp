#ifndef _MSC_VER
#include <unistd.h>
#endif
#include <thread>
#include <condition_variable>
#include <iostream>
#include <atomic>
#include <vector>
#include <string>
#include <csignal>

#include "rtc_base/logsinks.h"

#if USE_ROS
#include "ros/ros_log_sink.h"
#include "ros/ros_video_capture.h"
#include "signal_listener.h"
#else
#include "rtc/device_video_capturer.h"
#endif

#include "connection_settings.h"
#include "util.h"
#include "rtc/manager.h"
#include "sora/sora_server.h"
#include "p2p/p2p_server.h"

const size_t kDefaultMaxLogFileSize = 10 * 1024 * 1024;

int main(int argc, char* argv[])
{
  ConnectionSettings cs;

  bool is_daemon = false;
  bool use_p2p = false;
  bool use_sora = false;
  int log_level = rtc::LS_NONE;

  Util::parseArgs(argc, argv, is_daemon, use_p2p, use_sora, log_level, cs);

#ifndef _MSC_VER
  if (is_daemon)
  {
    if (daemon(1, 0) == -1)
    {
      std::cerr << "failed to launch momo daemon" << std::endl;
      return -1;
    }
  }
#endif


  rtc::LogMessage::LogToDebug((rtc::LoggingSeverity)log_level);
  rtc::LogMessage::LogTimestamps();
  rtc::LogMessage::LogThreads();

#if USE_ROS
  std::unique_ptr<rtc::LogSink> log_sink(new ROSLogSink());
  rtc::LogMessage::AddLogToStream(log_sink.get(), rtc::LS_INFO);
  std::unique_ptr<ROSVideoCapture> capturer(new ROSVideoCapture(cs));
#else
  std::unique_ptr<rtc::FileRotatingLogSink> log_sink(
      new rtc::FileRotatingLogSink("./", "webrtc_logs", kDefaultMaxLogFileSize, 10));
  if (!log_sink->Init())
  {
    RTC_LOG(LS_ERROR) << __FUNCTION__ << "Failed to open log file";
    log_sink.reset();
    return 1;
  }
  rtc::LogMessage::AddLogToStream(log_sink.get(), rtc::LS_INFO);
  std::unique_ptr<rtc::VideoSourceInterface<webrtc::VideoFrame>> capturer =
          DeviceVideoCapturer::Create(cs.getWidth(), cs.getHeight(), cs.framerate);
#endif

  std::unique_ptr<RTCManager> rtc_manager(new RTCManager(cs, std::move(capturer)));

  {
      boost::asio::io_context ioc{1};

      boost::asio::signal_set signals(ioc, SIGINT, SIGTERM);
      signals.async_wait([&](const boost::system::error_code&, int) {
          ioc.stop();
      });

      if (use_sora) {
        const boost::asio::ip::tcp::endpoint endpoint{boost::asio::ip::make_address("127.0.0.1"), static_cast<unsigned short>(cs.port)};
        std::make_shared<SoraServer>(ioc, endpoint, rtc_manager.get(), cs)->run();
      }

      if (use_p2p) {
        const boost::asio::ip::tcp::endpoint endpoint{boost::asio::ip::make_address("0.0.0.0"), static_cast<unsigned short>(cs.port)};
        std::make_shared<P2PServer>(ioc, endpoint, std::make_shared<std::string>(cs.p2p_document_root), rtc_manager.get(), cs)->run();
      }

      ioc.run();
  }

  rtc_manager = nullptr;

  return 0;
}
