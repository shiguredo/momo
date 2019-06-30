#ifndef RTC_MESSAGESENDER_H_
#define RTC_MESSAGESENDER_H_

#include <string>
#include "api/peer_connection_interface.h"

class RTCMessageSender
{
public:
  virtual void onIceConnectionStateChange(
          webrtc::PeerConnectionInterface::IceConnectionState new_state) = 0;
  virtual void onIceCandidate(const std::string sdp_mid,
          const int sdp_mlineindex, const std::string sdp) = 0;
  virtual void onCreateDescription(webrtc::SdpType type, const std::string sdp) = 0;
  virtual void onSetDescription(webrtc::SdpType type) = 0;
};

#endif
