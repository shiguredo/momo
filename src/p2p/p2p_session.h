#ifndef P2P_SESSION_H_
#define P2P_SESSION_H_

#include <cstdlib>
#include <functional>
#include <memory>
#include <string>

#include <boost/asio/bind_executor.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/strand.hpp>
#include <boost/beast/http/message.hpp>
#include <boost/beast/http/string_body.hpp>
#include <boost/beast/http/write.hpp>

#include "connection_settings.h"
#include "p2p_websocket_session.h"
#include "rtc/manager.h"
#include "util.h"

// 1つの HTTP リクエストを処理するためのクラス
class P2PSession : public std::enable_shared_from_this<P2PSession> {
  boost::asio::ip::tcp::socket socket_;
  boost::asio::strand<boost::asio::ip::tcp::socket::executor_type> strand_;
  boost::beast::flat_buffer buffer_;
  std::shared_ptr<std::string const> doc_root_;
  boost::beast::http::request<boost::beast::http::string_body> req_;
  std::shared_ptr<void> res_;

  RTCManager* rtc_manager_;
  ConnectionSettings conn_settings_;

 public:
  P2PSession(boost::asio::ip::tcp::socket socket,
             std::shared_ptr<std::string const> const& doc_root,
             RTCManager* rtc_manager,
             ConnectionSettings conn_settings);

  void run();

 private:
  void doRead();
  void onRead(boost::system::error_code ec, std::size_t bytes_transferred);

  void handleRequest();

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
            strand_, std::bind(&P2PSession::onWrite, shared_from_this(),
                               std::placeholders::_1, std::placeholders::_2,
                               sp->need_eof())));
  }

  void onWrite(boost::system::error_code ec,
               std::size_t bytes_transferred,
               bool close);
  void doClose();
};

#endif  // P2P_SESSION_H_
