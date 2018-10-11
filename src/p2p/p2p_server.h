#ifndef P2P_SERVER_H_
#define P2P_SERVER_H_

#include <nlohmann/json.hpp>
#include "CivetServer.h"

#include "rtc/manager.h"
#include "connection_settings.h"
#include "p2p_connection.h"

using json = nlohmann::json;

class P2PConnection;

class P2PServer : public CivetWebSocketHandler
{
public:
  P2PServer(CivetServer* server, RTCManager* rtc_manager,
          ConnectionSettings conn_settings);
  ~P2PServer();

  std::shared_ptr<RTCConnection> createConnection(struct mg_connection *conn);

  //websocket server
  bool handleData(
          CivetServer *server, struct mg_connection *conn,
          int bits, char *data, size_t data_len) override;
  void handleClose(
          CivetServer *server, const struct mg_connection *conn) override;

private:
  CivetServer* _server;
  RTCManager* _rtc_manager;

  ConnectionSettings _conn_settings;
  static const std::string _url;
  std::map<const struct mg_connection*, std::shared_ptr<P2PConnection>> _connections;
};
#endif