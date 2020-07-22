#ifndef P2P_SERVER_H_
#define P2P_SERVER_H_

#include <memory>
#include <string>

// nlohmann/json
#include <boost/asio/io_context.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/system/error_code.hpp>

#include "connection_settings.h"
#include "rtc/manager.h"
#include "util.h"

class P2PServer : public std::enable_shared_from_this<P2PServer> {
  boost::asio::io_context& ioc_;
  boost::asio::ip::tcp::acceptor acceptor_;
  boost::asio::ip::tcp::socket socket_;
  std::shared_ptr<std::string const> doc_root_;

  RTCManager* rtc_manager_;
  ConnectionSettings conn_settings_;

 public:
  P2PServer(boost::asio::io_context& ioc,
            boost::asio::ip::tcp::endpoint endpoint,
            std::shared_ptr<std::string const> const& doc_root,
            RTCManager* rtc_manager,
            ConnectionSettings conn_settings);

  void run();

 private:
  void doAccept();
  void onAccept(boost::system::error_code ec);
};

#endif
