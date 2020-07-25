#ifndef RTC_MANAGER_H_
#define RTC_MANAGER_H_

// WebRTC
#include <api/peer_connection_interface.h>
#include <pc/video_track_source.h>

#include "connection_settings.h"
#include "rtc_connection.h"
#include "rtc_data_manager.h"
#include "rtc_message_sender.h"
#include "scalable_track_source.h"
#include "video_track_receiver.h"

class RTCConnection;

class RTCManager {
 public:
  RTCManager(ConnectionSettings conn_settings,
             rtc::scoped_refptr<ScalableVideoTrackSource> video_track_source,
             VideoTrackReceiver* receiver);
  ~RTCManager();
  void SetDataManager(RTCDataManager* data_manager);
  std::shared_ptr<RTCConnection> CreateConnection(
      webrtc::PeerConnectionInterface::RTCConfiguration rtc_config,
      RTCMessageSender* sender);
  void InitTracks(RTCConnection* conn);

 private:
  rtc::scoped_refptr<webrtc::PeerConnectionFactoryInterface> factory_;
  rtc::scoped_refptr<webrtc::AudioTrackInterface> audio_track_;
  rtc::scoped_refptr<webrtc::VideoTrackInterface> video_track_;
  std::unique_ptr<rtc::Thread> network_thread_;
  std::unique_ptr<rtc::Thread> worker_thread_;
  std::unique_ptr<rtc::Thread> signaling_thread_;
  ConnectionSettings conn_settings_;
  VideoTrackReceiver* receiver_;
  RTCDataManager* data_manager_;
};

#endif
