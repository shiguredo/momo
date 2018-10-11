#include <iostream>
#include <nlohmann/json.hpp>

#include "p2p_connection.h"

using json = nlohmann::json;

P2PConnection::P2PConnection(RTCManager* rtc_manager, struct mg_connection* ws_conn) :
        _ws_conn(ws_conn)
{
  webrtc::PeerConnectionInterface::RTCConfiguration rtc_config;
  webrtc::PeerConnectionInterface::IceServers servers;
  webrtc::PeerConnectionInterface::IceServer ice_server;
  ice_server.uri = "stun:stun.l.google.com:19302";
  servers.push_back(ice_server);
  rtc_config.servers = servers;
  _connection = rtc_manager->createConnection(rtc_config, this);
}

void P2PConnection::onIceConnectionStateChange(IceConnectionState new_state)
{
  _rtc_state = new_state;
}

void P2PConnection::onIceCandidate(
        const std::string sdp_mid, const int sdp_mlineindex, const std::string sdp)
{
  json json_cand = {
    {"type", "candidate"}
  };
  json_cand["ice"] = {
    {"candidate", sdp},
    {"sdpMLineIndex", sdp_mlineindex},
    {"sdpMid", sdp_mid}
  };
  std::string str_cand = json_cand.dump();
  mg_websocket_write(_ws_conn, WEBSOCKET_OPCODE_TEXT, str_cand.c_str(), str_cand.length());
}

void P2PConnection::onCreateDescription(webrtc::SdpType type, const std::string sdp)
{
  json json_desc = {
    {"type", webrtc::SdpTypeToString(type)},
    {"sdp", sdp}
  };
  std::string str_desc = json_desc.dump();
  mg_websocket_write(_ws_conn, WEBSOCKET_OPCODE_TEXT, str_desc.c_str(), str_desc.length());
}