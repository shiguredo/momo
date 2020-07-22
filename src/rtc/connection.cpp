#include "connection.h"

// WebRTC
#include <rtc_base/logging.h>

// stats のコールバックを受け取るためのクラス
class RTCStatsCallback : public webrtc::RTCStatsCollectorCallback {
 public:
  typedef std::function<void(
      const rtc::scoped_refptr<const webrtc::RTCStatsReport>& report)>
      ResultCallback;

  static RTCStatsCallback* Create(ResultCallback result_callback) {
    return new rtc::RefCountedObject<RTCStatsCallback>(
        std::move(result_callback));
  }

  void OnStatsDelivered(
      const rtc::scoped_refptr<const webrtc::RTCStatsReport>& report) override {
    std::move(result_callback_)(report);
  }

 protected:
  RTCStatsCallback(ResultCallback result_callback)
      : result_callback_(std::move(result_callback)) {}
  ~RTCStatsCallback() override = default;

 private:
  ResultCallback result_callback_;
};

RTCConnection::~RTCConnection() {
  _connection->Close();
}

void RTCConnection::createOffer() {
  using RTCOfferAnswerOptions =
      webrtc::PeerConnectionInterface::RTCOfferAnswerOptions;
  RTCOfferAnswerOptions options = RTCOfferAnswerOptions();
  options.offer_to_receive_video =
      RTCOfferAnswerOptions::kOfferToReceiveMediaTrue;
  options.offer_to_receive_audio =
      RTCOfferAnswerOptions::kOfferToReceiveMediaTrue;
  _connection->CreateOffer(
      CreateSessionDescriptionObserver::Create(_sender, _connection), options);
}

void RTCConnection::setOffer(const std::string sdp) {
  webrtc::SdpParseError error;
  std::unique_ptr<webrtc::SessionDescriptionInterface> session_description =
      webrtc::CreateSessionDescription(webrtc::SdpType::kOffer, sdp, &error);
  if (!session_description) {
    RTC_LOG(LS_ERROR) << __FUNCTION__
                      << "Failed to create session description: "
                      << error.description.c_str()
                      << "\nline: " << error.line.c_str();
    return;
  }
  _connection->SetRemoteDescription(
      SetSessionDescriptionObserver::Create(session_description->GetType(),
                                            _sender),
      session_description.release());
}

void RTCConnection::createAnswer() {
  _connection->CreateAnswer(
      CreateSessionDescriptionObserver::Create(_sender, _connection),
      webrtc::PeerConnectionInterface::RTCOfferAnswerOptions());
}

void RTCConnection::setAnswer(const std::string sdp) {
  webrtc::SdpParseError error;
  std::unique_ptr<webrtc::SessionDescriptionInterface> session_description =
      webrtc::CreateSessionDescription(webrtc::SdpType::kAnswer, sdp, &error);
  if (!session_description) {
    RTC_LOG(LS_ERROR) << __FUNCTION__
                      << "Failed to create session description: "
                      << error.description.c_str()
                      << "\nline: " << error.line.c_str();
    return;
  }
  _connection->SetRemoteDescription(
      SetSessionDescriptionObserver::Create(session_description->GetType(),
                                            _sender),
      session_description.release());
}

void RTCConnection::addIceCandidate(const std::string sdp_mid,
                                    const int sdp_mlineindex,
                                    const std::string sdp) {
  webrtc::SdpParseError error;
  std::unique_ptr<webrtc::IceCandidateInterface> candidate(
      webrtc::CreateIceCandidate(sdp_mid, sdp_mlineindex, sdp, &error));
  if (!candidate.get()) {
    RTC_LOG(LS_ERROR) << "Can't parse received candidate message: "
                      << error.description.c_str()
                      << "\nline: " << error.line.c_str();
    return;
  }
  if (!_connection->AddIceCandidate(candidate.get())) {
    RTC_LOG(LS_WARNING) << __FUNCTION__
                        << "Failed to apply the received candidate : " << sdp;
    return;
  }
}

bool RTCConnection::setAudioEnabled(bool enabled) {
  return setMediaEnabled(getLocalAudioTrack(), enabled);
}

bool RTCConnection::setVideoEnabled(bool enabled) {
  return setMediaEnabled(getLocalVideoTrack(), enabled);
}

bool RTCConnection::isAudioEnabled() {
  return isMediaEnabled(getLocalAudioTrack());
}

bool RTCConnection::isVideoEnabled() {
  return isMediaEnabled(getLocalVideoTrack());
}

rtc::scoped_refptr<webrtc::MediaStreamInterface>
RTCConnection::getLocalStream() {
  return _connection->local_streams()->at(0);
}

rtc::scoped_refptr<webrtc::AudioTrackInterface>
RTCConnection::getLocalAudioTrack() {
  rtc::scoped_refptr<webrtc::MediaStreamInterface> local_stream =
      getLocalStream();
  if (!local_stream) {
    return nullptr;
  }

  if (local_stream->GetAudioTracks().size() > 0) {
    rtc::scoped_refptr<webrtc::AudioTrackInterface> audio_track(
        local_stream->GetAudioTracks()[0]);
    if (audio_track) {
      return audio_track;
    }
  }
  return nullptr;
}

rtc::scoped_refptr<webrtc::VideoTrackInterface>
RTCConnection::getLocalVideoTrack() {
  rtc::scoped_refptr<webrtc::MediaStreamInterface> local_stream =
      getLocalStream();
  if (!local_stream) {
    return nullptr;
  }

  if (local_stream->GetVideoTracks().size() > 0) {
    rtc::scoped_refptr<webrtc::VideoTrackInterface> video_track(
        local_stream->GetVideoTracks()[0]);
    if (video_track) {
      return video_track;
    }
  }
  return nullptr;
}

bool RTCConnection::setMediaEnabled(
    rtc::scoped_refptr<webrtc::MediaStreamTrackInterface> track,
    bool enabled) {
  if (track) {
    return track->set_enabled(enabled);
  }
  return false;
}

bool RTCConnection::isMediaEnabled(
    rtc::scoped_refptr<webrtc::MediaStreamTrackInterface> track) {
  if (track) {
    return track->enabled();
  }
  return false;
}

void RTCConnection::getStats(
    std::function<void(const rtc::scoped_refptr<const webrtc::RTCStatsReport>&)>
        callback) {
  _connection->GetStats(RTCStatsCallback::Create(std::move(callback)));
}

std::string RtpTransceiverDirectionToString(
    webrtc::RtpTransceiverDirection dir) {
  switch (dir) {
    case webrtc::RtpTransceiverDirection::kSendRecv:
      return "kSendRecv";
    case webrtc::RtpTransceiverDirection::kSendOnly:
      return "kSendOnly";
    case webrtc::RtpTransceiverDirection::kRecvOnly:
      return "kRecvOnly";
    case webrtc::RtpTransceiverDirection::kInactive:
      return "kInactive";
    case webrtc::RtpTransceiverDirection::kStopped:
      return "kStopped";
    default:
      return "UNKNOWN";
  }
}

void RTCConnection::setEncodingParameters(
    std::vector<webrtc::RtpEncodingParameters> encodings) {
  for (auto transceiver : _connection->GetTransceivers()) {
    RTC_LOG(LS_INFO) << "transceiver mid="
                     << transceiver->mid().value_or("nullopt") << " direction="
                     << RtpTransceiverDirectionToString(
                            transceiver->direction())
                     << " current_direction="
                     << (transceiver->current_direction()
                             ? RtpTransceiverDirectionToString(
                                   *transceiver->current_direction())
                             : "nullopt")
                     << " media_type="
                     << cricket::MediaTypeToString(transceiver->media_type())
                     << " sender_encoding_count="
                     << transceiver->sender()->GetParameters().encodings.size();
  }

  // setRD のあとの direction は recv only になる。
  // 現状 sender.track.streamIds を取れないので connection ID との比較もできない。
  // video upstream 持っているときは、ひとつめの video type transceiver を
  // 自分が send すべき transceiver と決め打ちする。
  rtc::scoped_refptr<webrtc::RtpTransceiverInterface> video_transceiver;
  for (auto transceiver : _connection->GetTransceivers()) {
    if (transceiver->media_type() == cricket::MediaType::MEDIA_TYPE_VIDEO) {
      video_transceiver = transceiver;
      break;
    }
  }

  if (video_transceiver == nullptr) {
    RTC_LOG(LS_ERROR) << "video transceiver not found";
    return;
  }

  rtc::scoped_refptr<webrtc::RtpSenderInterface> sender =
      video_transceiver->sender();
  webrtc::RtpParameters parameters = sender->GetParameters();
  parameters.encodings = std::move(encodings);
  sender->SetParameters(parameters);
}

rtc::scoped_refptr<webrtc::PeerConnectionInterface>
RTCConnection::getConnection() const {
  return _connection;
}
