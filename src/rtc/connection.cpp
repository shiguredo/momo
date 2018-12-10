#include "rtc_base/logging.h"

#include "connection.h"

RTCConnection::~RTCConnection()
{
  _manager->removeConnection(this);
  close();
}

void RTCConnection::createOffer()
{
  using RTCOfferAnswerOptions = webrtc::PeerConnectionInterface::RTCOfferAnswerOptions;
  RTCOfferAnswerOptions options = RTCOfferAnswerOptions();
  options.offer_to_receive_video = RTCOfferAnswerOptions::kOfferToReceiveMediaTrue;
  options.offer_to_receive_audio = RTCOfferAnswerOptions::kOfferToReceiveMediaTrue;
  _connection->CreateOffer(
          CreateSessionDescriptionObserver::Create(_sender, _connection), options);
}

void RTCConnection::setOffer(const std::string sdp)
{
  webrtc::SdpParseError error;
  std::unique_ptr<webrtc::SessionDescriptionInterface> session_description
          = webrtc::CreateSessionDescription(webrtc::SdpType::kOffer, sdp, &error);
  if (!session_description)
  {
    RTC_LOG(LS_ERROR) << __FUNCTION__ << "Failed to create session description: "
                      << error.description.c_str() << "\nline: " << error.line.c_str();
    return;
  }
  _connection->SetRemoteDescription(
          SetSessionDescriptionObserver::Create(
                  session_description->GetType(), _sender),
          session_description.release());
  _connection->CreateAnswer(
          CreateSessionDescriptionObserver::Create(_sender, _connection),
          webrtc::PeerConnectionInterface::RTCOfferAnswerOptions());
}

void RTCConnection::setAnswer(const std::string sdp)
{
  webrtc::SdpParseError error;
  std::unique_ptr<webrtc::SessionDescriptionInterface> session_description
          = webrtc::CreateSessionDescription(webrtc::SdpType::kAnswer, sdp, &error);
  if (!session_description)
  {
    RTC_LOG(LS_ERROR) << __FUNCTION__ << "Failed to create session description: "
                      << error.description.c_str() << "\nline: " << error.line.c_str();
    return;
  }
  _connection->SetRemoteDescription(
          SetSessionDescriptionObserver::Create(
                  session_description->GetType(), _sender),
          session_description.release());
}

void RTCConnection::addIceCandidate(
        const std::string sdp_mid, const int sdp_mlineindex, const std::string sdp)
{
  webrtc::SdpParseError error;
  std::unique_ptr<webrtc::IceCandidateInterface> candidate(
          webrtc::CreateIceCandidate(sdp_mid, sdp_mlineindex, sdp, &error));
  if (!candidate.get())
  {
    RTC_LOG(LS_ERROR) << "Can't parse received candidate message: "
                      << error.description.c_str() << "\nline: " << error.line.c_str();
    return;
  }
  if (!_connection->AddIceCandidate(candidate.get()))
  {
    RTC_LOG(LS_WARNING) << __FUNCTION__ << "Failed to apply the received candidate : " << sdp;
    return;
  }
}

bool RTCConnection::setAudioEnabled(bool enabled)
{
  return setMediaEnabled(getLocalAudioTrack(), enabled);
}

bool RTCConnection::setVideoEnabled(bool enabled)
{
  return setMediaEnabled(getLocalVideoTrack(), enabled);
}

bool RTCConnection::isAudioEnabled()
{
  return isMediaEnabled(getLocalAudioTrack());
}

bool RTCConnection::isVideoEnabled()
{
  return isMediaEnabled(getLocalVideoTrack());
}

rtc::scoped_refptr<webrtc::MediaStreamInterface> RTCConnection::getLocalStream()
{
  return _connection->local_streams()->at(0);
}

rtc::scoped_refptr<webrtc::AudioTrackInterface> RTCConnection::getLocalAudioTrack()
{
  rtc::scoped_refptr<webrtc::MediaStreamInterface> local_stream = getLocalStream();
  if (!local_stream)
  {
    return nullptr;
  }

  if (local_stream->GetAudioTracks().size() > 0)
  {
    rtc::scoped_refptr<webrtc::AudioTrackInterface> audio_track(local_stream->GetAudioTracks()[0]);
    if (audio_track)
    {
      return audio_track;
    }
  }
  return nullptr;
}

rtc::scoped_refptr<webrtc::VideoTrackInterface> RTCConnection::getLocalVideoTrack()
{
  rtc::scoped_refptr<webrtc::MediaStreamInterface> local_stream = getLocalStream();
  if (!local_stream)
  {
    return nullptr;
  }

  if (local_stream->GetVideoTracks().size() > 0)
  {
    rtc::scoped_refptr<webrtc::VideoTrackInterface> video_track(local_stream->GetVideoTracks()[0]);
    if (video_track)
    {
      return video_track;
    }
  }
  return nullptr;
}

bool RTCConnection::setMediaEnabled(
        rtc::scoped_refptr<webrtc::MediaStreamTrackInterface> track, bool enabled)
{
  if (track)
  {
    return track->set_enabled(enabled);
  }
  return false;
}

bool RTCConnection::isMediaEnabled(
        rtc::scoped_refptr<webrtc::MediaStreamTrackInterface> track)
{
  if (track)
  {
    return track->enabled();
  }
  return false;
}

void RTCConnection::close()
{
  _connection->Close();
}