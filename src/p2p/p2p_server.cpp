#include <iostream>

#include "p2p_server.h"

const std::string P2PServer::_url = "/ws";

P2PServer::P2PServer(
    CivetServer *server, RTCManager *rtc_manager, ConnectionSettings conn_settings) :
    _server(server), _rtc_manager(rtc_manager), _conn_settings(conn_settings)
{
  server->addWebSocketHandler(_url, this);
}

P2PServer::~P2PServer()
{
}

std::shared_ptr<RTCConnection> P2PServer::createConnection(
        struct mg_connection *conn) {
  std::shared_ptr<P2PConnection> p2p_conn =
          std::shared_ptr<P2PConnection>(new P2PConnection(_rtc_manager, conn));
  _connections.insert(std::make_pair(conn, p2p_conn));
  return p2p_conn->getRTCConnection();
}

bool P2PServer::handleData(
    CivetServer *server, struct mg_connection *conn,
    int bits, char *data, size_t data_len)
{
  std::string recv_string(data, data_len);
  json recv_message;
  try
  {
    recv_message = json::parse(recv_string);
  }
  catch (json::parse_error &e)
  {
    return false;
  }

  std::string type;
  try
  {
    type = recv_message["type"];
  }
  catch (json::type_error &e)
  {
    return false;
  }

  if (type == "offer")
  {
    std::string sdp;
    try
    {
      sdp = recv_message["sdp"];
    }
    catch (json::type_error &e)
    {
      return false;
    }
    std::shared_ptr<RTCConnection> rtc_conn = createConnection(conn);
    rtc_conn->setOffer(sdp);
  }
  else if (type == "answer")
  {
    std::shared_ptr<P2PConnection> p2p_conn = _connections[conn];
    if (!p2p_conn) {
      return false;
    }
    std::string sdp;
    try
    {
      sdp = recv_message["sdp"];
    }
    catch (json::type_error &e)
    {
      return false;
    }
    std::shared_ptr<RTCConnection> rtc_conn = p2p_conn->getRTCConnection();
    rtc_conn->setAnswer(sdp);
  }
  else if (type == "candidate")
  {
    std::shared_ptr<P2PConnection> p2p_conn = _connections[conn];
    if (!p2p_conn) {
      return false;
    }
    int sdp_mlineindex = 0;
    std::string sdp_mid, candidate;
    try
    {
      json ice = recv_message["ice"];
      sdp_mid = ice["sdpMid"];
      sdp_mlineindex = ice["sdpMLineIndex"];
      candidate = ice["candidate"];
    }
    catch (json::type_error &e)
    {
      return false;
    }
    std::shared_ptr<RTCConnection> rtc_conn = p2p_conn->getRTCConnection();
    rtc_conn->addIceCandidate(sdp_mid, sdp_mlineindex, candidate);
  }
  else if (type == "close")
  {
    _connections.erase(conn);
  } else {
    return false;
  }
  return true;
}

void P2PServer::handleClose(
    CivetServer *server, const struct mg_connection *conn)
{
  _connections.erase(conn);
}