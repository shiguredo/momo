#include "observer.h"

#include <iostream>

// WebRTC
#include <rtc_base/logging.h>

PeerConnectionObserver::~PeerConnectionObserver() {
  // Ayame 再接続時などには kIceConnectionDisconnected の前に破棄されているため
  ClearAllRegisteredTracks();
}

void PeerConnectionObserver::OnDataChannel(
    rtc::scoped_refptr<webrtc::DataChannelInterface> data_channel) {
  if (_data_manager != nullptr) {
    _data_manager->OnDataChannel(data_channel);
  }
}

void PeerConnectionObserver::OnStandardizedIceConnectionChange(
    webrtc::PeerConnectionInterface::IceConnectionState new_state) {
  RTC_LOG(LS_INFO) << __FUNCTION__ << " :" << new_state;
  if (new_state == webrtc::PeerConnectionInterface::IceConnectionState::
                       kIceConnectionDisconnected) {
    ClearAllRegisteredTracks();
  }
  if (_sender != nullptr) {
    _sender->onIceConnectionStateChange(new_state);
  }
}

void PeerConnectionObserver::OnIceCandidate(
    const webrtc::IceCandidateInterface* candidate) {
  std::string sdp;
  if (candidate->ToString(&sdp)) {
    if (_sender != nullptr) {
      _sender->onIceCandidate(candidate->sdp_mid(),
                              candidate->sdp_mline_index(), sdp);
    }
  } else {
    RTC_LOG(LS_ERROR) << "Failed to serialize candidate";
  }
}

void PeerConnectionObserver::OnTrack(
    rtc::scoped_refptr<webrtc::RtpTransceiverInterface> transceiver) {
  if (_receiver == nullptr)
    return;
  rtc::scoped_refptr<webrtc::MediaStreamTrackInterface> track =
      transceiver->receiver()->track();
  if (track->kind() == webrtc::MediaStreamTrackInterface::kVideoKind) {
    webrtc::VideoTrackInterface* video_track =
        static_cast<webrtc::VideoTrackInterface*>(track.get());
    _video_tracks.push_back(video_track);
    _receiver->AddTrack(video_track);
  }
}

void PeerConnectionObserver::OnRemoveTrack(
    rtc::scoped_refptr<webrtc::RtpReceiverInterface> receiver) {
  if (_receiver == nullptr)
    return;
  rtc::scoped_refptr<webrtc::MediaStreamTrackInterface> track =
      receiver->track();
  if (track->kind() == webrtc::MediaStreamTrackInterface::kVideoKind) {
    webrtc::VideoTrackInterface* video_track =
        static_cast<webrtc::VideoTrackInterface*>(track.get());
    _video_tracks.erase(
        std::remove_if(_video_tracks.begin(), _video_tracks.end(),
                       [video_track](const webrtc::VideoTrackInterface* track) {
                         return track == video_track;
                       }),
        _video_tracks.end());
    _receiver->RemoveTrack(video_track);
  }
}

void PeerConnectionObserver::ClearAllRegisteredTracks() {
  if (_receiver != nullptr) {
    for (webrtc::VideoTrackInterface* video_track : _video_tracks) {
      _receiver->RemoveTrack(video_track);
    }
  }
  _video_tracks.clear();
}
