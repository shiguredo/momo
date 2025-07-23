#include "peer_connection_observer.h"

#include <iostream>

// WebRTC
#include <rtc_base/logging.h>

PeerConnectionObserver::~PeerConnectionObserver() {
  // Ayame 再接続時などには kIceConnectionDisconnected の前に破棄されているため
  ClearAllRegisteredTracks();
}

RTCDataManager* PeerConnectionObserver::DataManager() {
  return data_manager_;
}

void PeerConnectionObserver::OnDataChannel(
    webrtc::scoped_refptr<webrtc::DataChannelInterface> data_channel) {
  if (data_manager_ != nullptr) {
    data_manager_->OnDataChannel(data_channel);
  }
}

void PeerConnectionObserver::OnStandardizedIceConnectionChange(
    webrtc::PeerConnectionInterface::IceConnectionState new_state) {
  RTC_LOG(LS_INFO) << __FUNCTION__ << " :" << new_state;
  if (new_state == webrtc::PeerConnectionInterface::IceConnectionState::
                       kIceConnectionDisconnected) {
    ClearAllRegisteredTracks();
  }
  if (sender_ != nullptr) {
    sender_->OnIceConnectionStateChange(new_state);
  }
}

void PeerConnectionObserver::OnIceCandidate(
    const webrtc::IceCandidateInterface* candidate) {
  std::string sdp;
  if (candidate->ToString(&sdp)) {
    if (sender_ != nullptr) {
      sender_->OnIceCandidate(candidate->sdp_mid(),
                              candidate->sdp_mline_index(), sdp);
    }
  } else {
    RTC_LOG(LS_ERROR) << "Failed to serialize candidate";
  }
}

void PeerConnectionObserver::OnTrack(
    webrtc::scoped_refptr<webrtc::RtpTransceiverInterface> transceiver) {
  if (receiver_ == nullptr)
    return;
  webrtc::scoped_refptr<webrtc::MediaStreamTrackInterface> track =
      transceiver->receiver()->track();
  if (track->kind() == webrtc::MediaStreamTrackInterface::kVideoKind) {
    webrtc::VideoTrackInterface* video_track =
        static_cast<webrtc::VideoTrackInterface*>(track.get());
    video_tracks_.push_back(video_track);
    receiver_->AddTrack(video_track);
  }
}

void PeerConnectionObserver::OnRemoveTrack(
    webrtc::scoped_refptr<webrtc::RtpReceiverInterface> receiver) {
  if (receiver_ == nullptr)
    return;
  webrtc::scoped_refptr<webrtc::MediaStreamTrackInterface> track =
      receiver->track();
  if (track->kind() == webrtc::MediaStreamTrackInterface::kVideoKind) {
    webrtc::VideoTrackInterface* video_track =
        static_cast<webrtc::VideoTrackInterface*>(track.get());
    video_tracks_.erase(
        std::remove_if(video_tracks_.begin(), video_tracks_.end(),
                       [video_track](const webrtc::VideoTrackInterface* track) {
                         return track == video_track;
                       }),
        video_tracks_.end());
    receiver_->RemoveTrack(video_track);
  }
}

void PeerConnectionObserver::ClearAllRegisteredTracks() {
  if (receiver_ != nullptr) {
    for (webrtc::VideoTrackInterface* video_track : video_tracks_) {
      receiver_->RemoveTrack(video_track);
    }
  }
  video_tracks_.clear();
}
