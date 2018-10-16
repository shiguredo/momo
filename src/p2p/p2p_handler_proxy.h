#ifndef P2P_HANDLER_PROXY_H_
#define P2P_HANDLER_PROXY_H_

// CivetServer の WebSocket ハンドラのプロキシ
//
// CivetServer の WebSocket ハンドラは、最低でも CivetServer と同じライフタイムじゃないといけないので、
// 先に本物のハンドラが死んでもダミーとして動き続けるようにする。

#include <memory>
#include "CivetServer.h"

class P2PHandlerProxy : public CivetWebSocketHandler
{
public:
  void setHandler(std::shared_ptr<CivetWebSocketHandler> handler);

  // CivetWebSocketHandler
  bool handleConnection(CivetServer *server,
                        const struct mg_connection *conn) override;
  void handleReadyState(CivetServer *server,
                        struct mg_connection *conn) override;
  bool handleData(CivetServer *server,
                  struct mg_connection *conn,
                  int bits,
                  char *data,
                  size_t data_len) override;
  void handleClose(CivetServer *server,
                   const struct mg_connection *conn) override;

private:
  std::weak_ptr<CivetWebSocketHandler> weak_handler_;
};

#endif // P2P_HANDLER_PROXY_H_
