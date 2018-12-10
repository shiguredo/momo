#ifndef CONNECTION_H_
#define CONNECTION_H_
#include "api/peerconnectioninterface.h"

#include "observer.h"
#include "manager.h"

class RTCManager;

class RTCConnection
{
public:
  RTCConnection(RTCManager *manager, RTCMessageSender *sender,
          rtc::scoped_refptr<webrtc::PeerConnectionInterface> connection)
          : _manager(manager), _sender(sender), _connection(connection) {};
  ~RTCConnection();
  void createOffer();
  void setOffer(const std::string sdp);
  void setAnswer(const std::string sdp);
  void addIceCandidate(const std::string sdp_mid,
        const int sdp_mlineindex, const std::string sdp);
  bool setAudioEnabled(bool enabled);
  bool setVideoEnabled(bool enabled);
  bool isAudioEnabled();
  bool isVideoEnabled();
  void close();

private:
  rtc::scoped_refptr<webrtc::MediaStreamInterface> getLocalStream();
  rtc::scoped_refptr<webrtc::AudioTrackInterface> getLocalAudioTrack();
  rtc::scoped_refptr<webrtc::VideoTrackInterface> getLocalVideoTrack();
  bool setMediaEnabled(
          rtc::scoped_refptr<webrtc::MediaStreamTrackInterface> track,
          bool enabled);
  bool isMediaEnabled(
          rtc::scoped_refptr<webrtc::MediaStreamTrackInterface> track);

  RTCManager *_manager;
  RTCMessageSender *_sender;
  rtc::scoped_refptr<webrtc::PeerConnectionInterface> _connection;
};
#endif