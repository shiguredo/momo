#ifndef SORA_SERVER_H_
#define SORA_SERVER_H_

#include <nlohmann/json.hpp>
#include "CivetServer.h"

#include "connection_settings.h"
#include "sora/sora_connection.h"

using json = nlohmann::json;

class SoraServer : public CivetHandler
{
public:
  SoraServer(
        CivetServer* server, RTCManager* rtc_manager,
        WebSocketClient* ws_client, ConnectionSettings conn_settings);
  ~SoraServer();

  bool connect();
  bool close();
  int writeOKwithJson(struct mg_connection *conn, json json_message);
  int writeFailed(struct mg_connection *conn);
  int writeBadRequest(struct mg_connection *conn);
  int writeMuteStatus(struct mg_connection *conn, std::shared_ptr<RTCConnection> rtc_conn);

  //web server
  bool handleGet(CivetServer *server, struct mg_connection *conn) override;
  bool handlePost(CivetServer *server, struct mg_connection *conn) override;

private:
  CivetServer* _server;
  RTCManager* _rtc_manager;
  WebSocketClient* _ws_client;
  std::unique_ptr<SoraConnection> _connection;

  static const std::string _urls[];
  ConnectionSettings _conn_settings;
};
#endif