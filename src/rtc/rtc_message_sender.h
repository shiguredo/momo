#ifndef RTC_MESSAGE_SENDER_H_
#define RTC_MESSAGE_SENDER_H_

#include <string>

// WebRTC
#include <api/peer_connection_interface.h>

class RTCMessageSender {
 public:
  virtual void onIceConnectionStateChange(
      webrtc::PeerConnectionInterface::IceConnectionState new_state) = 0;
  virtual void onIceCandidate(const std::string sdp_mid,
                              const int sdp_mlineindex,
                              const std::string sdp) = 0;
};

#endif
