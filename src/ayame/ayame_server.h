#ifndef AYAME_SERVER_H_
#define AYAME_SERVER_H_

#include <algorithm>
#include <cstdlib>
#include <functional>
#include <memory>
#include <string>

// Boost
#include <boost/asio/io_context.hpp>
#include <boost/asio/ip/tcp.hpp>

#include "ayame_websocket_client.h"
#include "connection_settings.h"
#include "rtc/manager.h"

// Ayame と接続する用のサーバ。
//
// サーバとは書いているが、これは SoraServer と同じような動きにするための名前付けで、
// 実際には Listen せず、単にサーバと同じ寿命を持つようにするだけのクラスになっている。
//
// そのため Ayame サーバは単に指定されたシグナリングサーバに WebSocket で
// 接続するだけのクラスとなる。
class AyameServer : public std::enable_shared_from_this<AyameServer> {
  boost::asio::deadline_timer timer_;
  std::shared_ptr<AyameWebsocketClient> ws_client_;

 public:
  AyameServer(boost::asio::io_context& ioc,
              RTCManager* rtc_manager,
              ConnectionSettings conn_settings);
  ~AyameServer();

  void run();

 private:
  void doWait();
  void onWait(boost::system::error_code ec);
};

#endif
