#include "p2p_server.h"

#include "p2p_session.h"
#include "util.h"

P2PServer::P2PServer(boost::asio::io_context& ioc,
                     boost::asio::ip::tcp::endpoint endpoint,
                     std::shared_ptr<std::string const> const& doc_root,
                     RTCManager* rtc_manager,
                     ConnectionSettings conn_settings)
    : ioc_(ioc),
      acceptor_(ioc),
      socket_(ioc),
      doc_root_(doc_root),
      rtc_manager_(rtc_manager),
      conn_settings_(conn_settings) {
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

void P2PServer::Run() {
  if (!acceptor_.is_open())
    return;
  DoAccept();
}

void P2PServer::DoAccept() {
  acceptor_.async_accept(socket_,
                         std::bind(&P2PServer::OnAccept, shared_from_this(),
                                   std::placeholders::_1));
}

void P2PServer::OnAccept(boost::system::error_code ec) {
  if (ec) {
    MOMO_BOOST_ERROR(ec, "accept");
  } else {
    std::make_shared<P2PSession>(ioc_, std::move(socket_), doc_root_,
                                 rtc_manager_, conn_settings_)
        ->Run();
  }

  DoAccept();
}
