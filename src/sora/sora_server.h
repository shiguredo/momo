#ifndef SORA_SERVER_H_
#define SORA_SERVER_H_

#include <boost/beast.hpp>
#include <boost/beast/websocket/ssl.hpp>
#include <boost/asio.hpp>
#include <boost/asio/ssl/stream.hpp>
#include <nlohmann/json.hpp>
#include <algorithm>
#include <cstdlib>
#include <functional>
#include <memory>
#include <string>

#include "rtc/manager.h"
#include "connection_settings.h"
#include "sora_websocket_client.h"

/*
Sora と接続する用のサーバ。

SoraServer には２種類の役割がある。

- 指定されたエンドポイントを listen して、HTTP 経由でリクエストされた処理に対応する
- 指定されたシグナリングサーバに WebSocket で接続し、シグナリングサーバからの offer に答えたりする
*/
class SoraServer : public std::enable_shared_from_this<SoraServer>
{
    boost::asio::ip::tcp::acceptor acceptor_;
    boost::asio::ip::tcp::socket socket_;

    RTCManager* rtc_manager_;
    ConnectionSettings conn_settings_;
    std::shared_ptr<SoraWebsocketClient> ws_client_;

public:
    SoraServer(
        boost::asio::io_context& ioc,
        boost::asio::ip::tcp::endpoint endpoint,
        RTCManager* rtc_manager,
        ConnectionSettings conn_settings);
    ~SoraServer();

    void run();

private:
    void doAccept();
    void onAccept(boost::system::error_code ec);
};

#endif
