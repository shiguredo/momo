#ifndef WEBSOCKET_CLIENT_H_
#define WEBSOCKET_CLIENT_H_

#include <websocketpp/config/asio_client.hpp>
#include <websocketpp/client.hpp>

#include "rtc/manager.h"

typedef websocketpp::connection_hdl WSPPHandle;
typedef websocketpp::client<websocketpp::config::asio_tls_client> WSPPClient;
typedef websocketpp::client<websocketpp::config::asio_client>::message_ptr WSPPMessagePtr;
typedef std::shared_ptr<boost::asio::ssl::context> WSPPContextPtr;

#include "ws_endpoint.h"

class SoraConnection;
class WebSocketEventListener;
class WebSocketEndpoint;

class WebSocketClient
{
public:
  WebSocketClient();
  ~WebSocketClient();

  std::shared_ptr<WebSocketEndpoint> createEndpoint(
          const std::string url, WebSocketEventListener *listener);

private:
  WSPPContextPtr onTLSInit(WSPPHandle handle);

  WSPPClient _client;
  std::shared_ptr<websocketpp::lib::thread> _thread;
};
#endif