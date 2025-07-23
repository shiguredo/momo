#include "rtc_connection.h"

// WebRTC
#include <api/peer_connection_interface.h>
#include <api/scoped_refptr.h>
#include <rtc_base/ref_counted_object.h>

// stats のコールバックを受け取るためのクラス
class RTCStatsCallback : public webrtc::RTCStatsCollectorCallback {
 public:
  typedef std::function<void(
      const webrtc::scoped_refptr<const webrtc::RTCStatsReport>& report)>
      ResultCallback;

  static RTCStatsCallback* Create(ResultCallback result_callback) {
    return new webrtc::RefCountedObject<RTCStatsCallback>(
        std::move(result_callback));
  }

  void OnStatsDelivered(
      const webrtc::scoped_refptr<const webrtc::RTCStatsReport>& report)
      override {
    std::move(result_callback_)(report);
  }

 protected:
  RTCStatsCallback(ResultCallback result_callback)
      : result_callback_(std::move(result_callback)) {}
  ~RTCStatsCallback() override = default;

 private:
  ResultCallback result_callback_;
};

// CreateSessionDescriptionObserver のコールバックを関数オブジェクトで扱えるようにするためのクラス
class CreateSessionDescriptionThunk
    : public webrtc::CreateSessionDescriptionObserver {
 public:
  typedef RTCConnection::OnCreateSuccessFunc OnSuccessFunc;
  typedef RTCConnection::OnCreateFailureFunc OnFailureFunc;

  static webrtc::scoped_refptr<CreateSessionDescriptionThunk> Create(
      OnSuccessFunc on_success,
      OnFailureFunc on_failure) {
    return webrtc::make_ref_counted<CreateSessionDescriptionThunk>(
        std::move(on_success), std::move(on_failure));
  }

 protected:
  CreateSessionDescriptionThunk(OnSuccessFunc on_success,
                                OnFailureFunc on_failure)
      : on_success_(std::move(on_success)),
        on_failure_(std::move(on_failure)) {}
  void OnSuccess(webrtc::SessionDescriptionInterface* desc) override {
    auto f = std::move(on_success_);
    if (f) {
      f(desc);
    }
  }
  void OnFailure(webrtc::RTCError error) override {
    RTC_LOG(LS_ERROR) << "Failed to create session description : "
                      << webrtc::ToString(error.type()) << ": "
                      << error.message();
    auto f = std::move(on_failure_);
    if (f) {
      f(error);
    }
  }

 private:
  OnSuccessFunc on_success_;
  OnFailureFunc on_failure_;
};

// SetSessionDescriptionObserver のコールバックを関数オブジェクトで扱えるようにするためのクラス
class SetSessionDescriptionThunk
    : public webrtc::SetSessionDescriptionObserver {
 public:
  typedef RTCConnection::OnSetSuccessFunc OnSuccessFunc;
  typedef RTCConnection::OnSetFailureFunc OnFailureFunc;

  static webrtc::scoped_refptr<SetSessionDescriptionThunk> Create(
      OnSuccessFunc on_success,
      OnFailureFunc on_failure) {
    return webrtc::make_ref_counted<SetSessionDescriptionThunk>(
        std::move(on_success), std::move(on_failure));
  }

 protected:
  SetSessionDescriptionThunk(OnSuccessFunc on_success, OnFailureFunc on_failure)
      : on_success_(std::move(on_success)),
        on_failure_(std::move(on_failure)) {}
  void OnSuccess() override {
    auto f = std::move(on_success_);
    if (f) {
      f();
    }
  }
  void OnFailure(webrtc::RTCError error) override {
    RTC_LOG(LS_ERROR) << "Failed to set session description : "
                      << webrtc::ToString(error.type()) << ": "
                      << error.message();
    auto f = std::move(on_failure_);
    if (f) {
      f(error);
    }
  }

 private:
  OnSuccessFunc on_success_;
  OnFailureFunc on_failure_;
};

RTCConnection::~RTCConnection() {
  connection_->Close();
}

void RTCConnection::CreateOffer(OnCreateSuccessFunc on_success,
                                OnCreateFailureFunc on_failure) {
  // CreateOffer を行うのは Ayame だけのため、ここで Offer の場合には DataChannel を作ることとした
  // Momo の性質上 ReOffer することは無いので問題ないと思われる
  RTCDataManager* data_manager = observer_->DataManager();
  if (data_manager != nullptr) {
    webrtc::DataChannelInit config;
    auto result = connection_->CreateDataChannelOrError("serial", &config);
    if (!result.ok()) {
      RTC_LOG(LS_ERROR) << "CreateDataChannel() failed: "
                        << result.error().message();
    }
    data_manager->OnDataChannel(result.MoveValue());
  }

  using RTCOfferAnswerOptions =
      webrtc::PeerConnectionInterface::RTCOfferAnswerOptions;
  RTCOfferAnswerOptions options = RTCOfferAnswerOptions();
  options.offer_to_receive_video =
      RTCOfferAnswerOptions::kOfferToReceiveMediaTrue;
  options.offer_to_receive_audio =
      RTCOfferAnswerOptions::kOfferToReceiveMediaTrue;

  auto with_set_local_desc = [this, on_success = std::move(on_success)](
                                 webrtc::SessionDescriptionInterface* desc) {
    std::string sdp;
    desc->ToString(&sdp);
    RTC_LOG(LS_INFO) << "Created session description : " << sdp;
    connection_->SetLocalDescription(
        SetSessionDescriptionThunk::Create(nullptr, nullptr).get(), desc);
    if (on_success) {
      on_success(desc);
    }
  };
  connection_->CreateOffer(
      CreateSessionDescriptionThunk::Create(std::move(with_set_local_desc),
                                            std::move(on_failure))
          .get(),
      options);
}

void RTCConnection::SetOffer(const std::string sdp,
                             OnSetSuccessFunc on_success,
                             OnSetFailureFunc on_failure) {
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
  connection_->SetRemoteDescription(
      SetSessionDescriptionThunk::Create(std::move(on_success),
                                         std::move(on_failure))
          .get(),
      session_description.release());
}

void RTCConnection::CreateAnswer(OnCreateSuccessFunc on_success,
                                 OnCreateFailureFunc on_failure) {
  auto with_set_local_desc = [this, on_success = std::move(on_success)](
                                 webrtc::SessionDescriptionInterface* desc) {
    std::string sdp;
    desc->ToString(&sdp);
    RTC_LOG(LS_INFO) << "Created session description : " << sdp;
    connection_->SetLocalDescription(
        SetSessionDescriptionThunk::Create(nullptr, nullptr).get(), desc);
    if (on_success) {
      on_success(desc);
    }
  };
  connection_->CreateAnswer(
      CreateSessionDescriptionThunk::Create(std::move(with_set_local_desc),
                                            std::move(on_failure))
          .get(),
      webrtc::PeerConnectionInterface::RTCOfferAnswerOptions());
}

void RTCConnection::SetAnswer(const std::string sdp,
                              OnSetSuccessFunc on_success,
                              OnSetFailureFunc on_failure) {
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
  connection_->SetRemoteDescription(
      SetSessionDescriptionThunk::Create(std::move(on_success),
                                         std::move(on_failure))
          .get(),
      session_description.release());
}

void RTCConnection::AddIceCandidate(const std::string sdp_mid,
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
  connection_->AddIceCandidate(
      std::move(candidate), [sdp](webrtc::RTCError error) {
        RTC_LOG(LS_WARNING)
            << __FUNCTION__ << " Failed to apply the received candidate. type="
            << webrtc::ToString(error.type()) << " message=" << error.message()
            << " sdp=" << sdp;
      });
}

bool RTCConnection::SetAudioEnabled(bool enabled) {
  return SetMediaEnabled(GetLocalAudioTrack(), enabled);
}

bool RTCConnection::SetVideoEnabled(bool enabled) {
  return SetMediaEnabled(GetLocalVideoTrack(), enabled);
}

bool RTCConnection::IsAudioEnabled() {
  return IsMediaEnabled(GetLocalAudioTrack());
}

bool RTCConnection::IsVideoEnabled() {
  return IsMediaEnabled(GetLocalVideoTrack());
}

webrtc::scoped_refptr<webrtc::MediaStreamInterface>
RTCConnection::GetLocalStream() {
  return webrtc::scoped_refptr<webrtc::MediaStreamInterface>(
      connection_->local_streams()->at(0));
}

webrtc::scoped_refptr<webrtc::AudioTrackInterface>
RTCConnection::GetLocalAudioTrack() {
  webrtc::scoped_refptr<webrtc::MediaStreamInterface> local_stream =
      GetLocalStream();
  if (!local_stream) {
    return nullptr;
  }

  if (local_stream->GetAudioTracks().size() > 0) {
    webrtc::scoped_refptr<webrtc::AudioTrackInterface> audio_track(
        local_stream->GetAudioTracks()[0]);
    if (audio_track) {
      return audio_track;
    }
  }
  return nullptr;
}

webrtc::scoped_refptr<webrtc::VideoTrackInterface>
RTCConnection::GetLocalVideoTrack() {
  webrtc::scoped_refptr<webrtc::MediaStreamInterface> local_stream =
      GetLocalStream();
  if (!local_stream) {
    return nullptr;
  }

  if (local_stream->GetVideoTracks().size() > 0) {
    webrtc::scoped_refptr<webrtc::VideoTrackInterface> video_track(
        local_stream->GetVideoTracks()[0]);
    if (video_track) {
      return video_track;
    }
  }
  return nullptr;
}

bool RTCConnection::SetMediaEnabled(
    webrtc::scoped_refptr<webrtc::MediaStreamTrackInterface> track,
    bool enabled) {
  if (track) {
    return track->set_enabled(enabled);
  }
  return false;
}

bool RTCConnection::IsMediaEnabled(
    webrtc::scoped_refptr<webrtc::MediaStreamTrackInterface> track) {
  if (track) {
    return track->enabled();
  }
  return false;
}

void RTCConnection::GetStats(
    std::function<void(
        const webrtc::scoped_refptr<const webrtc::RTCStatsReport>&)> callback) {
  connection_->GetStats(RTCStatsCallback::Create(std::move(callback)));
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

void RTCConnection::SetEncodingParameters(
    std::string mid,
    std::vector<webrtc::RtpEncodingParameters> encodings) {
  for (auto transceiver : connection_->GetTransceivers()) {
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
                     << webrtc::MediaTypeToString(transceiver->media_type())
                     << " sender_encoding_count="
                     << transceiver->sender()->GetParameters().encodings.size();
  }

  for (auto enc : encodings) {
    RTC_LOG(LS_INFO) << "SetEncodingParameters: rid=" << enc.rid
                     << " active=" << (enc.active ? "true" : "false")
                     << " max_framerate="
                     << (enc.max_framerate ? std::to_string(*enc.max_framerate)
                                           : std::string("nullopt"))
                     << " scale_resolution_down_by="
                     << (enc.scale_resolution_down_by
                             ? std::to_string(*enc.scale_resolution_down_by)
                             : std::string("nullopt"));
  }

  webrtc::scoped_refptr<webrtc::RtpTransceiverInterface> video_transceiver;
  if (mid.empty()) {
    // TODO(melpon): mid が手に入るようになったので、こっちの実装はそのうち消す

    // setRD のあとの direction は recv only になる。
    // 現状 sender.track.streamIds を取れないので connection ID との比較もできない。
    // video upstream 持っているときは、ひとつめの video type transceiver を
    // 自分が send すべき transceiver と決め打ちする。
    for (auto transceiver : connection_->GetTransceivers()) {
      if (transceiver->media_type() == webrtc::MediaType::VIDEO) {
        video_transceiver = transceiver;
        break;
      }
    }
  } else {
    for (auto transceiver : connection_->GetTransceivers()) {
      if (transceiver->mid() && *transceiver->mid() == mid) {
        video_transceiver = transceiver;
        break;
      }
    }
  }

  if (video_transceiver == nullptr) {
    RTC_LOG(LS_ERROR) << "video transceiver not found";
    return;
  }

  webrtc::scoped_refptr<webrtc::RtpSenderInterface> sender =
      video_transceiver->sender();
  webrtc::RtpParameters parameters = sender->GetParameters();
  parameters.encodings = encodings;
  sender->SetParameters(parameters);
  mid_ = *video_transceiver->mid();

  encodings_ = encodings;
}

void RTCConnection::ResetEncodingParameters() {
  if (encodings_.empty() || mid_.empty()) {
    return;
  }

  for (auto enc : encodings_) {
    RTC_LOG(LS_INFO) << "ResetEncodingParameters: rid=" << enc.rid
                     << " active=" << (enc.active ? "true" : "false")
                     << " max_framerate="
                     << (enc.max_framerate ? std::to_string(*enc.max_framerate)
                                           : std::string("nullopt"))
                     << " scale_resolution_down_by="
                     << (enc.scale_resolution_down_by
                             ? std::to_string(*enc.scale_resolution_down_by)
                             : std::string("nullopt"));
  }

  webrtc::scoped_refptr<webrtc::RtpTransceiverInterface> video_transceiver;
  for (auto transceiver : connection_->GetTransceivers()) {
    if (transceiver->mid() == mid_) {
      video_transceiver = transceiver;
      break;
    }
  }

  if (video_transceiver == nullptr) {
    RTC_LOG(LS_ERROR) << "video transceiver not found";
    return;
  }

  webrtc::scoped_refptr<webrtc::RtpSenderInterface> sender =
      video_transceiver->sender();
  webrtc::RtpParameters parameters = sender->GetParameters();
  std::vector<webrtc::RtpEncodingParameters> new_encodings = encodings_;

  // ssrc を上書きする
  for (auto& enc : new_encodings) {
    auto it =
        std::find_if(parameters.encodings.begin(), parameters.encodings.end(),
                     [&enc](const webrtc::RtpEncodingParameters& p) {
                       return p.rid == enc.rid;
                     });
    if (it == parameters.encodings.end()) {
      RTC_LOG(LS_WARNING) << "Specified rid [" << enc.rid << "] not found";
      return;
    }
    RTC_LOG(LS_INFO) << "Set ssrc: rid=" << enc.rid << " ssrc="
                     << (it->ssrc ? std::to_string(*it->ssrc)
                                  : std::string("nullopt"));
    enc.ssrc = it->ssrc;
  }
  parameters.encodings = new_encodings;
  sender->SetParameters(parameters);
}

webrtc::scoped_refptr<webrtc::PeerConnectionInterface>
RTCConnection::GetConnection() const {
  return connection_;
}
