#include "sora_server.h"

#include "sora_session.h"
#include "util.h"

SoraServer::SoraServer(boost::asio::io_context& ioc,
                       boost::asio::ip::tcp::endpoint endpoint,
                       std::shared_ptr<SoraClient> client,
                       RTCManager* rtc_manager,
                       SoraServerConfig config)
    : acceptor_(ioc),
      socket_(ioc),
      client_(client),
      rtc_manager_(rtc_manager),
      config_(std::move(config)) {
  boost::system::error_code ec;

  // Open the acceptor
  acceptor_.open(endpoint.protocol(), ec);
  if (ec) {
    MOMO_BOOST_ERROR(ec, "open");
    return;
  }

  // Allow address reuse
  acceptor_.set_option(boost::asio::socket_base::reuse_address(true), ec);
  if (ec) {
    MOMO_BOOST_ERROR(ec, "set_option");
    return;
  }

  // Bind to the server address
  acceptor_.bind(endpoint, ec);
  if (ec) {
    MOMO_BOOST_ERROR(ec, "bind");
    return;
  }

  // Start listening for connections
  acceptor_.listen(boost::asio::socket_base::max_listen_connections, ec);
  if (ec) {
    MOMO_BOOST_ERROR(ec, "listen");
    return;
  }
}

void SoraServer::Run() {
  if (!acceptor_.is_open())
    return;
  DoAccept();
}

void SoraServer::DoAccept() {
  acceptor_.async_accept(socket_,
                         std::bind(&SoraServer::OnAccept, shared_from_this(),
                                   std::placeholders::_1));
}

void SoraServer::OnAccept(boost::system::error_code ec) {
  if (ec) {
    MOMO_BOOST_ERROR(ec, "accept");
  } else {
    SoraSessionConfig config;
    SoraSession::Create(std::move(socket_), client_, rtc_manager_,
                        std::move(config))
        ->Run();
  }

  DoAccept();
}
