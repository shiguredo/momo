#include "ayame_server.h"

#include "util.h"

AyameServer::AyameServer(boost::asio::io_context& ioc,
                         RTCManager* rtc_manager,
                         ConnectionSettings conn_settings)
    : timer_(ioc),
      ws_client_(std::make_shared<AyameWebsocketClient>(ioc,
                                                        rtc_manager,
                                                        conn_settings)) {}

AyameServer::~AyameServer() {
  // これをやっておかないと循環参照でリークする
  ws_client_->release();
}

void AyameServer::run() {
  ws_client_->connect();
  doWait();
}

void AyameServer::doWait() {
  // あまり大きい時間にするのも怖いので５分に１回ぐらいは戻ってくるようにする
  timer_.expires_from_now(boost::posix_time::minutes(5));
  timer_.async_wait(std::bind(&AyameServer::onWait, shared_from_this(),
                              std::placeholders::_1));
}

void AyameServer::onWait(boost::system::error_code ec) {
  if (ec) {
    return;
  }

  doWait();
}
