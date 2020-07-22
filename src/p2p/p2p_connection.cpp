#include "p2p_connection.h"

#include <iostream>

// nlohmann/json
#include <nlohmann/json.hpp>

#include "util.h"

using json = nlohmann::json;
using IceConnectionState = webrtc::PeerConnectionInterface::IceConnectionState;

P2PConnection::P2PConnection(RTCManager* rtc_manager,
                             ConnectionSettings conn_settings,
                             std::function<void(std::string)> send)
    : _send(send) {
  webrtc::PeerConnectionInterface::RTCConfiguration rtc_config;
  webrtc::PeerConnectionInterface::IceServers servers;
  if (!conn_settings.no_google_stun) {
    webrtc::PeerConnectionInterface::IceServer ice_server;
    ice_server.uri = "stun:stun.l.google.com:19302";
    servers.push_back(ice_server);
  }
  rtc_config.servers = servers;
  _connection = rtc_manager->createConnection(rtc_config, this);
  rtc_manager->initTracks(_connection.get());
}

void P2PConnection::onIceConnectionStateChange(IceConnectionState new_state) {
  RTC_LOG(LS_INFO) << __FUNCTION__ << " rtc_state "
                   << Util::iceConnectionStateToString(_rtc_state) << " -> "
                   << Util::iceConnectionStateToString(new_state);

  _rtc_state = new_state;
}

void P2PConnection::onIceCandidate(const std::string sdp_mid,
                                   const int sdp_mlineindex,
                                   const std::string sdp) {
  RTC_LOG(LS_INFO) << __FUNCTION__;

  json json_cand = {{"type", "candidate"}};
  json_cand["ice"] = {{"candidate", sdp},
                      {"sdpMLineIndex", sdp_mlineindex},
                      {"sdpMid", sdp_mid}};
  std::string str_cand = json_cand.dump();
  _send(std::move(str_cand));
}

void P2PConnection::onCreateDescription(webrtc::SdpType type,
                                        const std::string sdp) {
  RTC_LOG(LS_INFO) << __FUNCTION__;

  json json_desc = {{"type", webrtc::SdpTypeToString(type)}, {"sdp", sdp}};
  std::string str_desc = json_desc.dump();
  _send(std::move(str_desc));
}

void P2PConnection::onSetDescription(webrtc::SdpType type) {
  RTC_LOG(LS_INFO) << __FUNCTION__
                   << " SdpType: " << webrtc::SdpTypeToString(type);
  if (type == webrtc::SdpType::kOffer) {
    _connection->createAnswer();
  }
}
