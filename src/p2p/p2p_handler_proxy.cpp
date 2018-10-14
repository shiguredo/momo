#include "p2p_handler_proxy.h"

void P2PHandlerProxy::setHandler(std::shared_ptr<CivetWebSocketHandler> handler) {
  weak_handler_ = handler;
}

bool P2PHandlerProxy::handleConnection(CivetServer *server, const struct mg_connection *conn) {
  auto handler = weak_handler_.lock();
  if (handler) {
    return handler->handleConnection(server, conn);
  } else {
    return CivetWebSocketHandler::handleConnection(server, conn);
  }
}

void P2PHandlerProxy::handleReadyState(CivetServer *server, struct mg_connection *conn) {
  auto handler = weak_handler_.lock();
  if (handler) {
    return handler->handleReadyState(server, conn);
  } else {
    return CivetWebSocketHandler::handleReadyState(server, conn);
  }
}

bool P2PHandlerProxy::handleData(CivetServer *server, struct mg_connection *conn, int bits, char *data, size_t data_len) {
  auto handler = weak_handler_.lock();
  if (handler) {
    return handler->handleData(server, conn, bits, data, data_len);
  } else {
    return CivetWebSocketHandler::handleData(server, conn, bits, data, data_len);
  }
}
void P2PHandlerProxy::handleClose(CivetServer *server, const struct mg_connection *conn) {
  auto handler = weak_handler_.lock();
  if (handler) {
    return handler->handleClose(server, conn);
  } else {
    return CivetWebSocketHandler::handleClose(server, conn);
  }
}
