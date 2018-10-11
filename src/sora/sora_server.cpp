#include "sora_server.h"
#include "util.h"

const std::string SoraServer::_urls[] = {
        "/connect", "/connect/status", "/close", "/mute", "/mute/status"};

SoraServer::SoraServer(
        CivetServer* server, RTCManager* rtc_manager,
        WebSocketClient* ws_client, ConnectionSettings conn_settings) :
        _server(server), _rtc_manager(rtc_manager),
        _ws_client(ws_client), _conn_settings(conn_settings)
{
  for (std::string url : _urls) {
    server->addHandler(url, this);
  }
  if (conn_settings.sora_auto_connect) {
    connect();
  }
}

SoraServer::~SoraServer()
{
  for (std::string url : _urls) {
    _server->removeHandler(url);
  }
  close();
}

bool SoraServer::connect()
{
  if (_connection)
  {
    return false;
  }
  _connection.reset(new SoraConnection(_rtc_manager, _ws_client, _conn_settings));
  return _connection->connect();
}

bool SoraServer::close()
{
  if (!_connection)
  {
    return false;
  }
  _connection = nullptr;
  return true;
}

int SoraServer::writeOKwithJson(struct mg_connection *conn, json json_message) {
  return mg_printf(conn, 
          "HTTP/1.1 200 OK\r\nContent-Type: "
          "application/json; charset=utf-8\r\n"
          "Connection: close\r\n"
          "\r\n%s",
          json_message.dump().c_str());
}

int SoraServer::writeFailed(struct mg_connection *conn)
{
  return mg_printf(conn,
          "HTTP/1.1 500 Internal Server Error\r\n"
          "Content-Type: application/json; charset=utf-8\r\n"
          "Connection: close\r\n"
          "\r\n");
}

int SoraServer::writeBadRequest(struct mg_connection *conn)
{
  return mg_printf(conn,
          "HTTP/1.1 400 Bad Request\r\n"
          "Content-Type: application/json; charset=utf-8\r\n"
          "Connection: close\r\n"
          "\r\n");
}

int SoraServer::writeMuteStatus(
        struct mg_connection *conn, std::shared_ptr<RTCConnection> rtc_conn)
{
  json json_message = {
    {"audio", !rtc_conn->isAudioEnabled()},
    {"video", !rtc_conn->isVideoEnabled()}
  };
  return writeOKwithJson(conn, json_message);
}

bool SoraServer::handleGet(CivetServer *server, struct mg_connection *conn)
{
  const struct mg_request_info *info = mg_get_request_info(conn);
  if (strcmp(info->local_uri, "/connect/status") == 0) {
    std::string state = RTCUtil::iceConnectionStateToString(
            _connection->getRTCConnectionState());
    json json_message = {
      {"state", state}
    };
    writeOKwithJson(conn, json_message);
    return true;
  } else if (strcmp(info->local_uri, "/mute/status") == 0) {
    std::shared_ptr<RTCConnection> rtc_conn =_connection->getRTCConnection();
    if (rtc_conn) {
      writeMuteStatus(conn, rtc_conn);
      return true;
    } else {
      writeFailed(conn);
      return true;
    }
  } 
  writeBadRequest(conn);
  return true;
}

bool SoraServer::handlePost(CivetServer *server, struct mg_connection *conn)
{
  const struct mg_request_info *info = mg_get_request_info(conn);
  if (strcmp(info->local_uri, "/connect") == 0) {
    json json_message = {
      {"result", connect()}
    };
    writeOKwithJson(conn, json_message);
    return true;
  } else if (strcmp(info->local_uri, "/close") == 0) {
    json json_message = {
      {"result", close()}
    };
    writeOKwithJson(conn, json_message);
    return true;
  } else {
    char post_data[8192];
    int post_data_len = mg_read(conn, post_data, sizeof(post_data));
    std::string post_string(post_data, post_data_len);
    
    json json_message;
    try {
      json_message = json::parse(post_string);
    } catch(json::parse_error& e) {
      writeBadRequest(conn);
      return true;
    }

    if (strcmp(info->local_uri, "/mute") == 0) {
      std::shared_ptr<RTCConnection> rtc_conn = _connection->getRTCConnection();
      if (!rtc_conn) {
        writeFailed(conn);
        return true;
      }
      try {
        bool audioMute = json_message["audio"];
        rtc_conn->setAudioEnabled(!audioMute);
      } catch(json::type_error& e) {}

      try {
        bool videoMute = json_message["video"];
        rtc_conn->setVideoEnabled(!videoMute);
      } catch(json::type_error& e) {}
      
      writeMuteStatus(conn, rtc_conn);
      return true;
    }
  }
  writeBadRequest(conn);
  return true;
}
