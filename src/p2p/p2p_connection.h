#ifndef P2P_CONNECTION_H_
#define P2P_CONNECTION_H_

#include "CivetServer.h"

#include "rtc/manager.h"
#include "rtc/connection.h"
#include "rtc/messagesender.h"

using IceConnectionState = webrtc::PeerConnectionInterface::IceConnectionState;

class P2PConnection : public RTCMessageSender
{
public:
  P2PConnection(RTCManager* rtc_manager, struct mg_connection* ws_conn);
  ~P2PConnection() {}

  IceConnectionState getRTCConnectionState() { return _rtc_state; }
  std::shared_ptr<RTCConnection> getRTCConnection() { return _connection; };

protected:
  //WebRTC
  void onIceConnectionStateChange(
          webrtc::PeerConnectionInterface::IceConnectionState new_state) override;
  void onIceCandidate(
          const std::string sdp_mid, const int sdp_mlineindex, const std::string sdp) override;
  void onCreateDescription(webrtc::SdpType type, const std::string sdp) override;
  void onSetDescription(webrtc::SdpType type) override {};

private:
  std::shared_ptr<RTCConnection> _connection;
  struct mg_connection* _ws_conn;
  IceConnectionState _rtc_state;
};
#endif