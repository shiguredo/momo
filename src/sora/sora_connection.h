#ifndef SORA_CONNECTION_H_
#define SORA_CONNECTION_H_
#include <atomic>

#include <nlohmann/json.hpp>
#include "rtc/connection.h"
#include "rtc/messagesender.h"

#include "connection_settings.h"
#include "ws_client/client.h"
#include "watchdog.h"

using json = nlohmann::json;
using IceConnectionState = webrtc::PeerConnectionInterface::IceConnectionState;

class SoraConnection : public RTCMessageSender, public WebSocketEventListener
{
public:
  SoraConnection(RTCManager *manager, WebSocketClient *wsclient, ConnectionSettings conn_settings);
  ~SoraConnection();

  IceConnectionState getRTCConnectionState() { return _rtc_state; }
  std::shared_ptr<RTCConnection> getRTCConnection();

  bool connect();
  bool disconnect();
  void reconnect(bool now);

  //websocket
  void onOpen() override;
  void onFail() override;
  void onMessage(std::string message) override;

protected:
  //WebRTC
  void onIceConnectionStateChange(
          webrtc::PeerConnectionInterface::IceConnectionState new_state) override;
  void onIceCandidate(
          const std::string sdp_mid, const int sdp_mlineindex, const std::string sdp) override;
  void onCreateDescription(webrtc::SdpType type, const std::string sdp) override;
  void onSetDescription(webrtc::SdpType type) override {};

  //watchdog
  void onWatchdogExpired();

private:
  void parseSoraMessage(std::string message);
  void createPeerFromConfig(json jconfig);
  void sendConnect();
  void sendPong();
  void sendAnswer(std::string answer);
  void sendCandidate(std::string candidate);

  WebSocketClient *_wsclient;
  std::shared_ptr<WebSocketEndpoint> _endpoint;

  RTCManager *_manager;
  std::shared_ptr<RTCConnection> _connection;

  std::unique_ptr<WatchDog> _watchdog;
  std::atomic<int> _retry_count;
  ConnectionSettings _conn_settings;
  IceConnectionState _rtc_state;
};
#endif