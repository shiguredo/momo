#ifndef RTC_UTIL_H_
#define RTC_UTIL_H_

#include "api/peerconnectioninterface.h"

#include "connection_settings.h"

class RTCUtil
{
  public:
    static void parseArgs(int argc, char *argv[], bool &is_daemon,
                          bool &use_p2p, bool &use_sora,
                          int &log_level, ConnectionSettings &cs);
    static std::string generateRundomChars();
    static std::string generateRundomChars(size_t length);
    static std::string iceConnectionStateToString(
            webrtc::PeerConnectionInterface::IceConnectionState state) ;
};

#endif