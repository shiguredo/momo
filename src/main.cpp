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

#include "CivetServer.h"

#if USE_ROS
#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include "ros/ros_video_capture.h"
#else
#include <boost/filesystem.hpp>
#endif

#include "connection_settings.h"
#include "util.h"
#include "rtc/manager.h"
#include "ws_client/client.h"
#include "sora/sora_server.h"
#include "p2p/p2p_server.h"

const size_t kDefaultMaxLogFileSize = 10 * 1024 * 1024;

struct signal_waiter {
  void set_running(bool running) {
    std::unique_lock<std::mutex> lock(_mutex);
    _running = running;
    _condition.notify_all();
  }

  void wait_until_running() {
    std::unique_lock<std::mutex> lock(_mutex);
    while (_running) {
      _condition.wait(lock);
    }
  }

private:
  std::mutex _mutex;
  bool _running;
  std::condition_variable _condition;

} _signal_waiter;

static void sigintHandler(int sig)
{
#if USE_ROS
  ros::shutdown();
#else
  _signal_waiter.set_running(false);
#endif
}

int main(int argc, char* argv[])
{
  ConnectionSettings cs;

  bool is_daemon = false;
  bool use_p2p = true;
  bool use_sora = false;
  int log_level = rtc::LS_NONE;

#if USE_ROS
  ros::init(argc, argv, "momo", ros::init_options::AnonymousName);
#else
  RTCUtil::parseArgs(argc, argv, is_daemon, use_p2p, use_sora, log_level, cs);
  _signal_waiter.set_running(true);
#endif
  signal(SIGINT, sigintHandler);

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

  std::unique_ptr<rtc::FileRotatingLogSink> log_sink(
      new rtc::FileRotatingLogSink("./", "webrtc_logs", kDefaultMaxLogFileSize, 10));
  if (!log_sink->Init())
  {
    RTC_LOG(LS_ERROR) << __FUNCTION__ << "Failed to open log file";
    log_sink.reset();
    return 1;
  }

  rtc::LogMessage::LogToDebug((rtc::LoggingSeverity)log_level);
  rtc::LogMessage::LogTimestamps();
  rtc::LogMessage::LogThreads();
  rtc::LogMessage::AddLogToStream(log_sink.get(), rtc::LS_INFO);

#if USE_ROS
  std::unique_ptr<cricket::VideoCapturer> capture(new ROSVideoCapture());
#else
  std::unique_ptr<cricket::VideoCapturer> capture = RTCManager::createVideoCapture();
#endif

  std::unique_ptr<RTCManager> rtc_manager(new RTCManager(cs, std::move(capture)));
  std::unique_ptr<WebSocketClient> ws_client(new WebSocketClient());

#if USE_ROS
  std::string currentPath(get_current_dir_name());
#else
  std::string currentPath = boost::filesystem::path(boost::filesystem::current_path()).string();
#endif
  std::string port_str = std::to_string(cs.p2p_port);
  const char* options[] = {
    "document_root", currentPath.c_str(),
    "listening_ports", port_str.c_str(),
    0
  };
  std::unique_ptr<CivetServer> server(new CivetServer(options));

  std::unique_ptr<SoraServer> sora_server;
  if (use_sora) {
    sora_server.reset(new SoraServer(
          server.get(), rtc_manager.get(), ws_client.get(), cs));
  }

  std::unique_ptr<P2PHandlerProxy> p2p_handler_proxy;
  std::shared_ptr<P2PServer> p2p_server;
  if (use_p2p) {
    p2p_handler_proxy.reset(new P2PHandlerProxy());
    p2p_server = P2PServer::create(server.get(), p2p_handler_proxy.get(), rtc_manager.get(), cs);
  }

#if USE_ROS
  ros::spin();
#else
  _signal_waiter.wait_until_running();
#endif

  p2p_server = nullptr;
  sora_server = nullptr;
  server = nullptr;
  // p2p_handler_proxy は必ず CivetServer より後に消すこと
  p2p_handler_proxy = nullptr;
  ws_client = nullptr;
  rtc_manager = nullptr;

  return 0;
}
