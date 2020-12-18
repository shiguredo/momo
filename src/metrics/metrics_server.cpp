#include "metrics_server.h"

MetricsServer::MetricsServer(boost::asio::io_context& ioc,
                             boost::asio::ip::tcp::endpoint endpoint,
                             RTCManager* rtc_manager,
                             std::shared_ptr<StatsCollector> stats_collector,
                             MetricsServerConfig config)
    : ioc_(ioc),
      acceptor_(ioc),
      socket_(ioc),
      rtc_manager_(rtc_manager),
      stats_collector_(stats_collector),
      config_(config) {
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

void MetricsServer::Run() {
  if (!acceptor_.is_open())
    return;
  DoAccept();
}

void MetricsServer::DoAccept() {
  acceptor_.async_accept(socket_,
                         std::bind(&MetricsServer::OnAccept, shared_from_this(),
                                   std::placeholders::_1));
}

void MetricsServer::OnAccept(boost::system::error_code ec) {
  if (ec) {
    MOMO_BOOST_ERROR(ec, "accept");
    return;
  }

  MetricsSessionConfig config;
  MetricsSession::Create(ioc_, std::move(socket_), rtc_manager_,
                         stats_collector_, std::move(config))
      ->Run();

  DoAccept();
}
