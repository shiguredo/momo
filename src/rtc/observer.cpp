#include <iostream>
#include "rtc_base/logging.h"

#include "observer.h"


void PeerConnectionObserver::OnIceConnectionChange(
        webrtc::PeerConnectionInterface::IceConnectionState new_state)
{
  _sender->onIceConnectionStateChange(new_state);
}

void PeerConnectionObserver::OnIceCandidate(const webrtc::IceCandidateInterface *candidate)
{
  std::string sdp;
  if (candidate->ToString(&sdp))
  {
    if (_sender != nullptr)
    {
      _sender->onIceCandidate(candidate->sdp_mid(), candidate->sdp_mline_index(), sdp);
    }
  }
  else
  {
    RTC_LOG(LS_ERROR) << "Failed to serialize candidate";
  }
}

void CreateSessionDescriptionObserver::OnSuccess(webrtc::SessionDescriptionInterface *desc)
{
  std::string sdp;
  desc->ToString(&sdp);
  RTC_LOG(LS_INFO) << "Created session description : " << sdp;
  _connection->SetLocalDescription(
          SetSessionDescriptionObserver::Create(desc->GetType(), _sender), desc);
  if (_sender != nullptr)
  {
    _sender->onCreateDescription(desc->GetType(), sdp);
  }
}

void CreateSessionDescriptionObserver::OnFailure(webrtc::RTCError error)
{
  RTC_LOG(LS_ERROR) << "Failed to create session description : "
                    << ToString(error.type()) << ": " << error.message();
}

void SetSessionDescriptionObserver::OnSuccess()
{
  RTC_LOG(LS_INFO) << "Set local description success!";
  if (_sender != nullptr)
  {
    _sender->onSetDescription(_type);
  }
}

void SetSessionDescriptionObserver::OnFailure(webrtc::RTCError error)
{
  RTC_LOG(LS_ERROR) << "Failed to set local description : "
                    << ToString(error.type()) << ": " << error.message();
}