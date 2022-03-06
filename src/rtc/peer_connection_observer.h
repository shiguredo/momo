#ifndef PEER_CONNECTION_OBSERVER_H_
#define PEER_CONNECTION_OBSERVER_H_

// WebRTC
#include <api/peer_connection_interface.h>

#include "rtc_data_manager.h"
#include "rtc_message_sender.h"
#include "video_track_receiver.h"

class PeerConnectionObserver : public webrtc::PeerConnectionObserver {
 public:
  PeerConnectionObserver(RTCMessageSender* sender,
                         VideoTrackReceiver* receiver,
                         RTCDataManager* data_manager)
      : sender_(sender), receiver_(receiver), data_manager_(data_manager) {}
  ~PeerConnectionObserver();

  RTCDataManager* DataManager();

 private:
  void OnSignalingChange(
      webrtc::PeerConnectionInterface::SignalingState new_state) override {}
  void OnDataChannel(
      rtc::scoped_refptr<webrtc::DataChannelInterface> data_channel) override;
  void OnRenegotiationNeeded() override {}
  void OnStandardizedIceConnectionChange(
      webrtc::PeerConnectionInterface::IceConnectionState new_state) override;
  void OnIceGatheringChange(
      webrtc::PeerConnectionInterface::IceGatheringState new_state) override{};
  void OnIceCandidate(const webrtc::IceCandidateInterface* candidate) override;
  void OnTrack(
      rtc::scoped_refptr<webrtc::RtpTransceiverInterface> transceiver) override;
  void OnRemoveTrack(
      rtc::scoped_refptr<webrtc::RtpReceiverInterface> receiver) override;

 private:
  void ClearAllRegisteredTracks();

  RTCMessageSender* sender_;
  VideoTrackReceiver* receiver_;
  RTCDataManager* data_manager_;
  std::vector<webrtc::VideoTrackInterface*> video_tracks_;
};

#endif
