#ifndef METRICS_SERVER_H_
#define METRICS_SERVER_H_

#include <memory>
#include <string>

#include <boost/asio/io_context.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/system/error_code.hpp>

#include "metrics_session.h"
#include "rtc/rtc_manager.h"
#include "stats_collector.h"
#include "util.h"

struct MetricsServerConfig {};

class MetricsServer : public std::enable_shared_from_this<MetricsServer> {
  MetricsServer(boost::asio::io_context& ioc,
                boost::asio::ip::tcp::endpoint endpoint,
                RTCManager* rtc_manager,
                std::shared_ptr<StatsCollector> stats_collector,
                MetricsServerConfig config);

 public:
  static std::shared_ptr<MetricsServer> Create(
      boost::asio::io_context& ioc,
      boost::asio::ip::tcp::endpoint endpoint,
      RTCManager* rtc_manager,
      std::shared_ptr<StatsCollector> stats_collector,
      MetricsServerConfig config) {
    return std::shared_ptr<MetricsServer>(new MetricsServer(
        ioc, endpoint, rtc_manager, stats_collector, std::move(config)));
  }
  void Run();

 private:
  void DoAccept();
  void OnAccept(boost::system::error_code ec);

 private:
  boost::asio::io_context& ioc_;
  boost::asio::ip::tcp::acceptor acceptor_;
  boost::asio::ip::tcp::socket socket_;

  RTCManager* rtc_manager_;
  MetricsServerConfig config_;
  std::shared_ptr<StatsCollector> stats_collector_;
};

#endif
