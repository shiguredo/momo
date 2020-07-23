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
#include "sora_client.h"

class SoraServer : public std::enable_shared_from_this<SoraServer> {
  boost::asio::ip::tcp::acceptor acceptor_;
  boost::asio::ip::tcp::socket socket_;

  std::shared_ptr<SoraClient> client_;
  RTCManager* rtc_manager_;
  ConnectionSettings conn_settings_;

 public:
  // Listen する版
  SoraServer(boost::asio::io_context& ioc,
             boost::asio::ip::tcp::endpoint endpoint,
             std::shared_ptr<SoraClient> client,
             RTCManager* rtc_manager,
             ConnectionSettings conn_settings);
  void Run();

 private:
  void DoAccept();
  void OnAccept(boost::system::error_code ec);
};

#endif
