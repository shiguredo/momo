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
      : _sender(sender), _receiver(receiver), _data_manager(data_manager){};
  ~PeerConnectionObserver();

 protected:
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

  RTCMessageSender* _sender;
  VideoTrackReceiver* _receiver;
  RTCDataManager* _data_manager;
  std::vector<webrtc::VideoTrackInterface*> _video_tracks;
};

#endif
