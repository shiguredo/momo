#ifndef P2P_WEBSOCKET_SESSION_H_
#define P2P_WEBSOCKET_SESSION_H_

#include <boost/asio/ip/tcp.hpp>
#include <boost/beast/core/multi_buffer.hpp>
#include <boost/beast/http/message.hpp>
#include <boost/beast/http/string_body.hpp>
#include <boost/system/error_code.hpp>
#include <cstdlib>
#include <memory>
#include <string>

#include "connection_settings.h"
#include "p2p_connection.h"
#include "rtc/manager.h"
#include "util.h"
#include "watchdog.h"
#include "ws/websocket.h"

class P2PWebsocketSession
    : public std::enable_shared_from_this<P2PWebsocketSession> {
  std::unique_ptr<Websocket> ws_;
  boost::beast::multi_buffer sending_buffer_;

  WatchDog watchdog_;

  RTCManager* rtc_manager_;
  ConnectionSettings conn_settings_;
  std::shared_ptr<P2PConnection> connection_;

 public:
  P2PWebsocketSession(boost::asio::io_context& ioc,
                      RTCManager* rtc_manager,
                      ConnectionSettings conn_settings);
  ~P2PWebsocketSession();
  static std::shared_ptr<P2PWebsocketSession> make_shared(
      boost::asio::io_context& ioc,
      boost::asio::ip::tcp::socket socket,
      RTCManager* rtc_manager,
      ConnectionSettings conn_settings);
  void run(boost::beast::http::request<boost::beast::http::string_body> req);

 private:
  void onWatchdogExpired();
  void doAccept(
      boost::beast::http::request<boost::beast::http::string_body> req);
  void onAccept(boost::system::error_code ec);

  void onRead(boost::system::error_code ec,
              std::size_t bytes_transferred,
              std::string recv_string);
};

#endif  // P2P_WEBSOCKET_SESSION_H_
