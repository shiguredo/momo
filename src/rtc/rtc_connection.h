#ifndef RTC_CONNECTION_H_
#define RTC_CONNECTION_H_

// WebRTC
#include <api/peer_connection_interface.h>

#include "peer_connection_observer.h"

class RTCConnection {
 public:
  RTCConnection(RTCMessageSender* sender,
                std::unique_ptr<PeerConnectionObserver> observer,
                rtc::scoped_refptr<webrtc::PeerConnectionInterface> connection)
      : _sender(sender),
        _observer(std::move(observer)),
        _connection(connection){};
  ~RTCConnection();

  typedef std::function<void(webrtc::SessionDescriptionInterface*)>
      OnCreateSuccessFunc;
  typedef std::function<void(webrtc::RTCError)> OnCreateFailureFunc;

  typedef std::function<void()> OnSetSuccessFunc;
  typedef std::function<void(webrtc::RTCError)> OnSetFailureFunc;

  void createOffer(OnCreateSuccessFunc on_success = nullptr,
                   OnCreateFailureFunc on_failure = nullptr);
  void setOffer(const std::string sdp,
                OnSetSuccessFunc on_success = nullptr,
                OnSetFailureFunc on_failure = nullptr);
  void createAnswer(OnCreateSuccessFunc on_success = nullptr,
                    OnCreateFailureFunc on_failure = nullptr);
  void setAnswer(const std::string sdp,
                 OnSetSuccessFunc on_success = nullptr,
                 OnSetFailureFunc on_failure = nullptr);
  void addIceCandidate(const std::string sdp_mid,
                       const int sdp_mlineindex,
                       const std::string sdp);
  bool setAudioEnabled(bool enabled);
  bool setVideoEnabled(bool enabled);
  bool isAudioEnabled();
  bool isVideoEnabled();

  void getStats(
      std::function<void(
          const rtc::scoped_refptr<const webrtc::RTCStatsReport>&)> callback);

  void setEncodingParameters(
      std::vector<webrtc::RtpEncodingParameters> encodings);

  rtc::scoped_refptr<webrtc::PeerConnectionInterface> getConnection() const;

 private:
  rtc::scoped_refptr<webrtc::MediaStreamInterface> getLocalStream();
  rtc::scoped_refptr<webrtc::AudioTrackInterface> getLocalAudioTrack();
  rtc::scoped_refptr<webrtc::VideoTrackInterface> getLocalVideoTrack();
  bool setMediaEnabled(
      rtc::scoped_refptr<webrtc::MediaStreamTrackInterface> track,
      bool enabled);
  bool isMediaEnabled(
      rtc::scoped_refptr<webrtc::MediaStreamTrackInterface> track);

  RTCMessageSender* _sender;
  std::unique_ptr<PeerConnectionObserver> _observer;
  rtc::scoped_refptr<webrtc::PeerConnectionInterface> _connection;
};

#endif
