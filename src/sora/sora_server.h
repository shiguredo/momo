#ifndef SORA_SERVER_H_
#define SORA_SERVER_H_

#include <algorithm>
#include <cstdlib>
#include <functional>
#include <memory>
#include <string>

// Boost
#include <boost/asio/io_context.hpp>
#include <boost/asio/ip/tcp.hpp>

#include "connection_settings.h"
#include "rtc/rtc_manager.h"
#include "sora_websocket_client.h"

// Sora と接続する用のサーバ。
//
// SoraServer には２種類の役割がある。
//
// - 指定されたエンドポイントを listen して、HTTP 経由でリクエストされた処理に対応する
// - 指定されたシグナリングサーバに WebSocket で接続し、シグナリングサーバからの offer に答えたりする
class SoraServer : public std::enable_shared_from_this<SoraServer> {
  bool listen_;
  boost::asio::deadline_timer timer_;

  boost::asio::ip::tcp::acceptor acceptor_;
  boost::asio::ip::tcp::socket socket_;

  RTCManager* rtc_manager_;
  ConnectionSettings conn_settings_;
  std::shared_ptr<SoraWebsocketClient> ws_client_;

 public:
  // Listen しない版
  SoraServer(boost::asio::io_context& ioc,
             RTCManager* rtc_manager,
             ConnectionSettings conn_settings);
  // Listen する版
  SoraServer(boost::asio::io_context& ioc,
             boost::asio::ip::tcp::endpoint endpoint,
             RTCManager* rtc_manager,
             ConnectionSettings conn_settings);
  ~SoraServer();

  void run();

 private:
  void doAccept();
  void onAccept(boost::system::error_code ec);
  void doWait();
  void onWait(boost::system::error_code ec);
};

#endif
