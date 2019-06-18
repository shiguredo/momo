#ifndef RTC_MANAGER_H_
#define RTC_MANAGER_H_
#include "api/peer_connection_interface.h"
#include "pc/video_track_source.h"

#include "connection.h"
#include "connection_settings.h"
#include "scalable_track_source.h"

class RTCManager
{
public:
  RTCManager(ConnectionSettings conn_settings,
             rtc::scoped_refptr<ScalableVideoTrackSource> video_track_source);
  ~RTCManager();
  std::shared_ptr<RTCConnection> createConnection(
          webrtc::PeerConnectionInterface::RTCConfiguration rtc_config,
          RTCMessageSender *sender);

private:
  rtc::scoped_refptr<webrtc::PeerConnectionFactoryInterface> _factory;
  rtc::scoped_refptr<webrtc::VideoTrackSourceInterface> _video_source;
  std::unique_ptr<rtc::Thread> _networkThread;
  std::unique_ptr<rtc::Thread> _workerThread;
  std::unique_ptr<rtc::Thread> _signalingThread;
  ConnectionSettings _conn_settings;
};
#endif
