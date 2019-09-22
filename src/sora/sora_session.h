#ifndef SORA_SESSION_H_
#define SORA_SESSION_H_

#include <algorithm>
#include <boost/asio/bind_executor.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/strand.hpp>
#include <boost/beast/core/flat_buffer.hpp>
#include <boost/beast/http/message.hpp>
#include <boost/beast/http/string_body.hpp>
#include <boost/beast/http/write.hpp>
#include <cstdlib>
#include <functional>
#include <memory>
#include <nlohmann/json.hpp>
#include <string>

#include "connection_settings.h"
#include "rtc/manager.h"
#include "sora_websocket_client.h"

// HTTP の１回のリクエストに対して答えるためのクラス
class SoraSession : public std::enable_shared_from_this<SoraSession> {
  boost::asio::ip::tcp::socket socket_;
  boost::asio::strand<boost::asio::ip::tcp::socket::executor_type> strand_;
  boost::beast::flat_buffer buffer_;
  boost::beast::http::request<boost::beast::http::string_body> req_;
  std::shared_ptr<void> res_;

  RTCManager* rtc_manager_;
  std::shared_ptr<SoraWebsocketClient> ws_client_;
  ConnectionSettings conn_settings_;

 public:
  SoraSession(boost::asio::ip::tcp::socket socket,
              RTCManager* rtc_manager,
              std::shared_ptr<SoraWebsocketClient> ws_client,
              ConnectionSettings conn_settings);

  void run();

 private:
  void doRead();

  void onRead(boost::system::error_code ec, std::size_t bytes_transferred);
  void onWrite(boost::system::error_code ec,
               std::size_t bytes_transferred,
               bool close);
  void doClose();

 private:
  static boost::beast::http::response<boost::beast::http::string_body>
  createOKwithJson(
      const boost::beast::http::request<boost::beast::http::string_body>& req,
      nlohmann::json json_message);

  template <class Body, class Fields>
  void sendResponse(boost::beast::http::response<Body, Fields> msg) {
    auto sp = std::make_shared<boost::beast::http::response<Body, Fields>>(
        std::move(msg));

    // msg オブジェクトは書き込みが完了するまで生きている必要があるので、
    // メンバに入れてライフタイムを延ばしてやる
    res_ = sp;

    // Write the response
    boost::beast::http::async_write(
        socket_, *sp,
        boost::asio::bind_executor(
            strand_, std::bind(&SoraSession::onWrite, shared_from_this(),
                               std::placeholders::_1, std::placeholders::_2,
                               sp->need_eof())));
  }
};

#endif  // SORA_SESSION_H_
