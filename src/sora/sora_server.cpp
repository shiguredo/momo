#include "sora_server.h"

#include "util.h"
#include "sora_session.h"

SoraServer::SoraServer(
    boost::asio::io_context& ioc,
    boost::asio::ip::tcp::endpoint endpoint,
    RTCManager* rtc_manager,
    ConnectionSettings conn_settings)
    : acceptor_(ioc)
    , socket_(ioc)
    , rtc_manager_(rtc_manager)
    , conn_settings_(conn_settings)
    , ws_client_(std::make_shared<SoraWebsocketClient>(ioc, rtc_manager, conn_settings))
{
    boost::system::error_code ec;

    // Open the acceptor
    acceptor_.open(endpoint.protocol(), ec);
    if (ec)
    {
        MOMO_BOOST_ERROR(ec, "open");
        return;
    }

    // Allow address reuse
    acceptor_.set_option(boost::asio::socket_base::reuse_address(true), ec);
    if (ec)
    {
        MOMO_BOOST_ERROR(ec, "set_option");
        return;
    }

    // Bind to the server address
    acceptor_.bind(endpoint, ec);
    if (ec)
    {
        MOMO_BOOST_ERROR(ec, "bind");
        return;
    }

    // Start listening for connections
    acceptor_.listen(
        boost::asio::socket_base::max_listen_connections, ec);
    if (ec)
    {
        MOMO_BOOST_ERROR(ec, "listen");
        return;
    }

}

SoraServer::~SoraServer() {
    // これをやっておかないと循環参照でリークする
    ws_client_->release();
}

void SoraServer::run()
{
    if (!acceptor_.is_open())
        return;

    if (conn_settings_.sora_auto_connect) {
        ws_client_->connect();
    }
    doAccept();
}

void SoraServer::doAccept()
{
    acceptor_.async_accept(
        socket_,
        std::bind(
            &SoraServer::onAccept,
            shared_from_this(),
            std::placeholders::_1));
}

void SoraServer::onAccept(boost::system::error_code ec)
{
    if (ec)
    {
        MOMO_BOOST_ERROR(ec, "accept");
    }
    else
    {
        // Create the session and run it
        std::make_shared<SoraSession>(std::move(socket_), rtc_manager_, ws_client_, conn_settings_)->run();
    }

    // Accept another connection
    doAccept();
}
