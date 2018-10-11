#ifndef WEBSOCKET_ENDPOINT_H_
#define WEBSOCKET_ENDPOINT_H_

#include <string>
#include "client.h"

class WebSocketEventListener
{
public:
  virtual void onOpen() {}
  virtual void onClose() {}
  virtual void onFail() {}
  virtual void onMessage(std::string message) {}
};

class WebSocketEndpoint
{
public:
  enum WebSocketState {
    kWebSocketConnecting,
    kWebSocketConnected,
    kWebSocketDisconnecting,
    kWebSocketClosed,
    kWebSocketFailed
  };

  WebSocketEndpoint(
          WSPPClient *client,
          WSPPClient::connection_ptr con_ptr,
          WebSocketEventListener *listener) :
          _client(client),
          _con_ptr(con_ptr),
          _listener(listener),
          _wsstate(WebSocketState::kWebSocketConnecting) {}
  ~WebSocketEndpoint() {};

  WebSocketState getState() { return _wsstate; }

  void sendText(std::string message);
  void close();
  void close(
          websocketpp::close::status::value code,
          std::string reason);
  void release();

  //websocket
  void onOpen(WSPPHandle handle);
  void onClose(WSPPHandle handle);
  void onFail(WSPPHandle handle);
  void onMessage(WSPPHandle handle, WSPPMessagePtr msg_ptr);

private:
  WSPPClient *_client;
  WSPPClient::connection_ptr _con_ptr;
  WebSocketEventListener *_listener;
  WebSocketState _wsstate;
};
#endif