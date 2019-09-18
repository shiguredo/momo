#ifndef AYAME_SERVER_H_
#define AYAME_SERVER_H_

#include <algorithm>
#include <boost/asio/io_context.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <cstdlib>
#include <functional>
#include <memory>
#include <string>

#include "ayame_websocket_client.h"
#include "connection_settings.h"
#include "rtc/manager.h"

/*
  Ayame と接続する用のサーバ。
  Ayame サーバの役割は以下の通り
    - 指定されたシグナリングサーバに WebSocket で接続
    - websocket の挙動は `./ayame_websocket_client` に記述している
*/
class AyameServer : public std::enable_shared_from_this<AyameServer> {
  boost::asio::ip::tcp::acceptor acceptor_;
  boost::asio::ip::tcp::socket socket_;

  RTCManager* rtc_manager_;
  ConnectionSettings conn_settings_;
  std::shared_ptr<AyameWebsocketClient> ws_client_;

 public:
  AyameServer(boost::asio::io_context& ioc,
              boost::asio::ip::tcp::endpoint endpoint,
              RTCManager* rtc_manager,
              ConnectionSettings conn_settings);
  ~AyameServer();

  void run();

 private:
  void doAccept();
  void onAccept(boost::system::error_code ec);
};

#endif
