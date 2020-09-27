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

#include "rtc/rtc_manager.h"
#include "sora_client.h"

struct SoraServerConfig {};

class SoraServer : public std::enable_shared_from_this<SoraServer> {
  SoraServer(boost::asio::io_context& ioc,
             boost::asio::ip::tcp::endpoint endpoint,
             std::shared_ptr<SoraClient> client,
             RTCManager* rtc_manager,
             SoraServerConfig config);

 public:
  static std::shared_ptr<SoraServer> Create(
      boost::asio::io_context& ioc,
      boost::asio::ip::tcp::endpoint endpoint,
      std::shared_ptr<SoraClient> client,
      RTCManager* rtc_manager,
      SoraServerConfig config) {
    return std::shared_ptr<SoraServer>(
        new SoraServer(ioc, endpoint, client, rtc_manager, std::move(config)));
  }

  void Run();

 private:
  void DoAccept();
  void OnAccept(boost::system::error_code ec);

 private:
  boost::asio::ip::tcp::acceptor acceptor_;
  boost::asio::ip::tcp::socket socket_;

  std::shared_ptr<SoraClient> client_;
  RTCManager* rtc_manager_;
  SoraServerConfig config_;
};

#endif
