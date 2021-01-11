#ifndef METRICS_SESSION_H_
#define METRICS_SESSION_H_

#include <cstdlib>
#include <functional>
#include <memory>
#include <string>

#include <boost/asio/bind_executor.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/strand.hpp>
#include <boost/beast/core/flat_buffer.hpp>
#include <boost/beast/http/message.hpp>
#include <boost/beast/http/string_body.hpp>
#include <boost/beast/http/write.hpp>
#include <boost/json.hpp>

#include "rtc/rtc_manager.h"
#include "stats_collector.h"
#include "util.h"

struct MetricsSessionConfig {};

// 1つの HTTP リクエストを処理するためのクラス
class MetricsSession : public std::enable_shared_from_this<MetricsSession> {
  MetricsSession(boost::asio::io_context& ioc,
                 boost::asio::ip::tcp::socket socket,
                 RTCManager* rtc_manager,
                 std::shared_ptr<StatsCollector> stats_collector,
                 MetricsSessionConfig config);

 public:
  static std::shared_ptr<MetricsSession> Create(
      boost::asio::io_context& ioc,
      boost::asio::ip::tcp::socket socket,
      RTCManager* rtc_manager,
      std::shared_ptr<StatsCollector> stats_collector,
      MetricsSessionConfig config) {
    return std::shared_ptr<MetricsSession>(
        new MetricsSession(ioc, std::move(socket), rtc_manager, stats_collector,
                           std::move(config)));
  }
  void Run();

 private:
  void DoRead();
  void OnRead(boost::system::error_code ec, std::size_t bytes_transferred);

  static boost::beast::http::response<boost::beast::http::string_body>
  CreateOKWithJSON(
      const boost::beast::http::request<boost::beast::http::string_body>& req,
      boost::json::value json_message);

  template <class Body, class Fields>
  void SendResponse(boost::beast::http::response<Body, Fields> msg) {
    auto sp = std::make_shared<boost::beast::http::response<Body, Fields>>(
        std::move(msg));

    // msg オブジェクトは書き込みが完了するまで生きている必要があるので、
    // メンバに入れてライフタイムを延ばしてやる
    res_ = sp;

    // Write the response
    boost::beast::http::async_write(
        socket_, *sp,
        std::bind(&MetricsSession::OnWrite, shared_from_this(),
                  std::placeholders::_1, std::placeholders::_2,
                  sp->need_eof()));
  }

  void OnWrite(boost::system::error_code ec,
               std::size_t bytes_transferred,
               bool close);
  void DoClose();

 private:
  boost::asio::io_context& ioc_;
  boost::asio::ip::tcp::socket socket_;
  boost::asio::strand<boost::asio::ip::tcp::socket::executor_type> strand_;
  boost::beast::flat_buffer buffer_;
  boost::beast::http::request<boost::beast::http::string_body> req_;
  std::shared_ptr<void> res_;

  RTCManager* rtc_manager_;
  MetricsSessionConfig config_;
  std::shared_ptr<StatsCollector> stats_collector_;
};

#endif  // METRICS_SESSION_H_
