#ifndef P2P_WEBSOCKET_SESSION_H_
#define P2P_WEBSOCKET_SESSION_H_

#include <nlohmann/json.hpp>
#include <algorithm>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <memory>
#include <string>

#include "rtc/manager.h"
#include "ws/websocket.h"
#include "connection_settings.h"
#include "util.h"
#include "p2p_connection.h"

class P2PWebsocketSession : public std::enable_shared_from_this<P2PWebsocketSession>
{
    std::unique_ptr<Websocket> ws_;
    boost::beast::multi_buffer sending_buffer_;

    RTCManager* rtc_manager_;
    ConnectionSettings conn_settings_;
    std::shared_ptr<P2PConnection> connection_;

public:
    P2PWebsocketSession(RTCManager* rtc_manager, ConnectionSettings conn_settings);
    ~P2PWebsocketSession();
    static std::shared_ptr<P2PWebsocketSession> make_shared(boost::asio::ip::tcp::socket socket, RTCManager* rtc_manager, ConnectionSettings conn_settings);
    void run(boost::beast::http::request<boost::beast::http::string_body> req);

private:
    void doAccept(boost::beast::http::request<boost::beast::http::string_body> req);
    void onAccept(boost::system::error_code ec);

    void onRead(boost::system::error_code ec, std::size_t bytes_transferred, std::string recv_string);
};

#endif // P2P_WEBSOCKET_SESSION_H_
