#ifndef RTC_UTIL_H_
#define RTC_UTIL_H_

#include "api/peerconnectioninterface.h"

class RTCUtil
{
  public:
    static std::string generateRundomChars();
    static std::string generateRundomChars(size_t length);
    static std::string iceConnectionStateToString(
            webrtc::PeerConnectionInterface::IceConnectionState state) ;
};

#endif