#ifndef PEERCONNECTIONOBSERVER_H_
#define PEERCONNECTIONOBSERVER_H_

#include "api/peer_connection_interface.h"
#include "messagesender.h"
#include "video_track_receiver.h"

class PeerConnectionObserver : public webrtc::PeerConnectionObserver {
 public:
  PeerConnectionObserver(RTCMessageSender* sender, VideoTrackReceiver* receiver)
      : _sender(sender), _receiver(receiver){};
  ~PeerConnectionObserver();

 protected:
  void OnSignalingChange(
      webrtc::PeerConnectionInterface::SignalingState new_state) override {}
  void OnDataChannel(
      rtc::scoped_refptr<webrtc::DataChannelInterface> data_channel) override {}
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
  std::vector<webrtc::VideoTrackInterface*> _video_tracks;
};

class CreateSessionDescriptionObserver
    : public webrtc::CreateSessionDescriptionObserver {
 public:
  static CreateSessionDescriptionObserver* Create(
      RTCMessageSender* sender,
      webrtc::PeerConnectionInterface* connection) {
    return new rtc::RefCountedObject<CreateSessionDescriptionObserver>(
        sender, connection);
  }

 protected:
  CreateSessionDescriptionObserver(RTCMessageSender* sender,
                                   webrtc::PeerConnectionInterface* connection)
      : _sender(sender), _connection(connection){};
  void OnSuccess(webrtc::SessionDescriptionInterface* desc) override;
  void OnFailure(webrtc::RTCError error) override;

 private:
  RTCMessageSender* _sender;
  webrtc::PeerConnectionInterface* _connection;
};

class SetSessionDescriptionObserver
    : public webrtc::SetSessionDescriptionObserver {
 public:
  static SetSessionDescriptionObserver* Create(webrtc::SdpType type,
                                               RTCMessageSender* sender) {
    return new rtc::RefCountedObject<SetSessionDescriptionObserver>(type,
                                                                    sender);
  }

 protected:
  SetSessionDescriptionObserver(webrtc::SdpType type, RTCMessageSender* sender)
      : _type(type), _sender(sender){};
  void OnSuccess() override;
  void OnFailure(webrtc::RTCError error) override;

 private:
  RTCMessageSender* _sender;
  webrtc::SdpType _type;
};
#endif