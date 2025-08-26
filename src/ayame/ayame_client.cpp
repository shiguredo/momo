#include "ayame_client.h"

// boost
#include <boost/beast/websocket/stream.hpp>
#include <boost/json.hpp>

// WebRTC
#include <api/peer_connection_interface.h>
#include <api/rtp_transceiver_interface.h>

#include <algorithm>

#include "momo_version.h"
#include "ssl_verifier.h"
#include "url_parts.h"
#include "util.h"

namespace {
// 映像補助コーデック
const std::vector<std::string> kVideoAuxiliaryCodecs = {
    "RTX", "RED", "ULPFEC", "FLEXFEC-03"
};

// 音声補助コーデック  
const std::vector<std::string> kAudioAuxiliaryCodecs = {
    "TELEPHONE-EVENT", "CN"
};

// 大文字変換ヘルパー関数
std::string ToUpperCase(const std::string& str) {
  std::string result = str;
  std::transform(result.begin(), result.end(), result.begin(), ::toupper);
  return result;
}

// コーデックが補助コーデックかどうかを判定
bool IsAuxiliaryCodec(const std::string& codec_name,
                      const std::vector<std::string>& auxiliary_codecs) {
  return std::find(auxiliary_codecs.begin(), auxiliary_codecs.end(),
                   codec_name) != auxiliary_codecs.end();
}
}  // namespace

bool AyameClient::ParseURL(URLParts& parts) const {
  std::string url = config_.signaling_url;

  if (!URLParts::Parse(url, parts)) {
    throw std::exception();
  }

  std::string default_port;
  if (parts.scheme == "wss") {
    return true;
  } else if (parts.scheme == "ws") {
    return false;
  } else {
    throw std::exception();
  }
}

void AyameClient::GetStats(
    std::function<void(
        const webrtc::scoped_refptr<const webrtc::RTCStatsReport>&)> callback) {
  if (connection_ &&
      (rtc_state_ == webrtc::PeerConnectionInterface::IceConnectionState::
                         kIceConnectionConnected ||
       rtc_state_ == webrtc::PeerConnectionInterface::IceConnectionState::
                         kIceConnectionCompleted)) {
    connection_->GetStats(std::move(callback));
  } else {
    callback(nullptr);
  }
}

AyameClient::AyameClient(boost::asio::io_context& ioc,
                         RTCManager* manager,
                         AyameClientConfig config)
    : ioc_(ioc),
      manager_(manager),
      retry_count_(0),
      config_(std::move(config)),
      watchdog_(ioc, std::bind(&AyameClient::OnWatchdogExpired, this)) {
  Reset();
}

AyameClient::~AyameClient() {
  destructed_ = true;
  // ここで OnIceConnectionStateChange が呼ばれる
  connection_ = nullptr;
}

void AyameClient::Reset() {
  connection_ = nullptr;
  is_send_offer_ = false;
  has_is_exist_user_flag_ = false;
  ice_servers_.clear();

  URLParts parts;
  if (ParseURL(parts)) {
    ws_.reset(new Websocket(Websocket::ssl_tag(), ioc_, config_.insecure,
                            config_.client_cert, config_.client_key));
  } else {
    ws_.reset(new Websocket(ioc_));
  }
}

void AyameClient::Connect() {
  RTC_LOG(LS_INFO) << __FUNCTION__;

  watchdog_.Enable(30);

  ws_->Connect(config_.signaling_url,
               std::bind(&AyameClient::OnConnect, shared_from_this(),
                         std::placeholders::_1));
}

void AyameClient::ReconnectAfter() {
  int interval = 5 * (2 * retry_count_);
  if (interval > 30) {
    interval = 30;
  }
  RTC_LOG(LS_INFO) << __FUNCTION__ << " reconnect after " << interval << " sec";

  watchdog_.Enable(interval);
  retry_count_++;
}

void AyameClient::OnWatchdogExpired() {
  RTC_LOG(LS_WARNING) << __FUNCTION__;

  RTC_LOG(LS_INFO) << __FUNCTION__ << " reconnecting...:";
  Reset();
  Connect();
}

void AyameClient::OnConnect(boost::system::error_code ec) {
  if (ec) {
    ReconnectAfter();
    return MOMO_BOOST_ERROR(ec, "Handshake");
  }

  DoRead();

  DoRegister();
}

void AyameClient::DoRead() {
  ws_->Read(std::bind(&AyameClient::OnRead, shared_from_this(),
                      std::placeholders::_1, std::placeholders::_2,
                      std::placeholders::_3));
}

void AyameClient::DoRegister() {
  boost::json::value json_message = {
      {"type", "register"},
      {"clientId", Util::GenerateRandomChars()},
      {"roomId", config_.room_id},
      {"ayameClient", MomoVersion::GetClientName()},
      {"libwebrtc", MomoVersion::GetLibwebrtcName()},
      {"environment", MomoVersion::GetEnvironmentName()},
  };
  if (config_.client_id != "") {
    json_message.as_object()["clientId"] = config_.client_id;
  }
  if (config_.signaling_key != "") {
    json_message.as_object()["key"] = config_.signaling_key;
  }
  ws_->WriteText(boost::json::serialize(json_message));
}

void AyameClient::DoSendPong() {
  boost::json::value json_message = {{"type", "pong"}};
  ws_->WriteText(boost::json::serialize(json_message));
}

void AyameClient::SetIceServersFromConfig(boost::json::value json_message) {
  // 返却されてきた iceServers を セットする
  if (json_message.as_object().count("iceServers") != 0) {
    auto jservers = json_message.at("iceServers");
    if (jservers.is_array()) {
      for (auto jserver : jservers.as_array()) {
        webrtc::PeerConnectionInterface::IceServer ice_server;
        if (jserver.as_object().count("username") != 0) {
          ice_server.username = jserver.at("username").as_string().c_str();
        }
        if (jserver.as_object().count("credential") != 0) {
          ice_server.password = jserver.at("credential").as_string().c_str();
        }
        auto jurls = jserver.at("urls");
        for (const auto url : jurls.as_array()) {
          ice_server.urls.push_back(url.as_string().c_str());
          RTC_LOG(LS_INFO) << __FUNCTION__
                           << ": iceserver.url=" << url.as_string();
        }
        ice_servers_.push_back(ice_server);
      }
    }
  }
  if (ice_servers_.empty() && !config_.no_google_stun) {
    // accept 時に iceServers が返却されてこなかった場合 google の stun server を用いる
    webrtc::PeerConnectionInterface::IceServer ice_server;
    ice_server.uri = "stun:stun.l.google.com:19302";
    ice_servers_.push_back(ice_server);
  }
}

void AyameClient::CreatePeerConnection() {
  webrtc::PeerConnectionInterface::RTCConfiguration rtc_config;

  rtc_config.servers = ice_servers_;
  connection_ = manager_->CreateConnection(rtc_config, this);
  manager_->InitTracks(connection_.get());

  // InitTracks で Transceiver が作成された後に SetCodecPreferences を呼ぶ
  // Transceiver の存在を確認してから呼び出す
  auto pc = connection_->GetConnection();
  if (pc) {
    auto transceivers = pc->GetTransceivers();
    if (!transceivers.empty()) {
      SetCodecPreferences();
    } else {
      RTC_LOG(LS_WARNING) << "No transceivers available after InitTracks";
    }
  }
}

void AyameClient::SetCodecPreferences() {
  if (config_.video_codec_type.empty() && config_.audio_codec_type.empty()) {
    return;
  }

  auto pc = connection_->GetConnection();
  if (!pc) {
    RTC_LOG(LS_ERROR) << "PeerConnection is null";
    return;
  }

  // PeerConnectionFactory から GetRtpSenderCapabilities を使ってコーデック一覧を取得
  auto factory = manager_->GetFactory();
  if (!factory) {
    RTC_LOG(LS_ERROR) << "PeerConnectionFactory is null";
    return;
  }

  auto transceivers = pc->GetTransceivers();
  
  // Transceiver が存在しない場合は警告を出して終了
  if (transceivers.empty()) {
    RTC_LOG(LS_ERROR) << "No transceivers found when trying to set codec preferences";
    return;
  }
  
  for (auto transceiver : transceivers) {
    std::vector<webrtc::RtpCodecCapability> filtered_codecs;

    // PeerConnectionFactory から capabilities を取得
    webrtc::RtpCapabilities capabilities;
    if (transceiver->media_type() == webrtc::MediaType::VIDEO) {
      capabilities =
          factory->GetRtpSenderCapabilities(webrtc::MediaType::VIDEO);
    } else if (transceiver->media_type() == webrtc::MediaType::AUDIO) {
      capabilities =
          factory->GetRtpSenderCapabilities(webrtc::MediaType::AUDIO);
    } else {
      continue;
    }

    auto current_codecs = capabilities.codecs;

    // 現在のコーデック一覧が空の場合はスキップ
    if (current_codecs.empty()) {
      RTC_LOG(LS_WARNING) << "No codec capabilities available for transceiver";
      continue;
    }

    if (transceiver->media_type() == webrtc::MediaType::VIDEO &&
        !config_.video_codec_type.empty()) {
      // 映像コーデックのフィルタリング
      std::string target_codec = ToUpperCase(config_.video_codec_type);

      for (const auto& codec : current_codecs) {
        std::string codec_name = ToUpperCase(codec.name);

        // 指定されたコーデックまたは補助的なコーデックは残す
        if (codec_name == target_codec ||
            IsAuxiliaryCodec(codec_name, kVideoAuxiliaryCodecs)) {
          filtered_codecs.push_back(codec);
        }
      }
    } else if (transceiver->media_type() == webrtc::MediaType::AUDIO &&
               !config_.audio_codec_type.empty()) {
      // 音声コーデックのフィルタリング
      std::string target_codec = ToUpperCase(config_.audio_codec_type);

      for (const auto& codec : current_codecs) {
        std::string codec_name = ToUpperCase(codec.name);

        // 指定されたコーデックまたは補助的なコーデックは残す
        if (codec_name == target_codec ||
            IsAuxiliaryCodec(codec_name, kAudioAuxiliaryCodecs)) {
          filtered_codecs.push_back(codec);
        }
      }
    }

    if (!filtered_codecs.empty()) {
      auto error = transceiver->SetCodecPreferences(filtered_codecs);
      if (!error.ok()) {
        RTC_LOG(LS_ERROR) << "Failed to set codec preferences: "
                          << error.message();
      } else {
        RTC_LOG(LS_INFO) << "Successfully set codec preferences for "
                         << (transceiver->media_type() ==
                                     webrtc::MediaType::VIDEO
                                 ? "video"
                                 : "audio");
      }
    }
  }
}

void AyameClient::Close() {
  ws_->Close(std::bind(&AyameClient::OnClose, shared_from_this(),
                       std::placeholders::_1));
}

// WebSocket が閉じられたときのコールバック
void AyameClient::OnClose(boost::system::error_code ec) {
  if (ec)
    MOMO_BOOST_ERROR(ec, "Close");
  // retry_count_ は ReconnectAfter(); が以前に呼ばれている場合はインクリメントされている可能性がある。
  // WebSocket につないでいない時間をなるべく短くしたいので、
  // WebSocket を閉じたときは一度インクリメントされている可能性のある retry_count_ を0 にして
  // OnWatchdogExpired(); が発火して再接続が行われるまでの時間を最小にしておく。
  retry_count_ = 0;
  // WebSocket 接続がちゃんと閉じられたら ReconnectAfter(); を発火する。
  // ReconnectAfter(); によって OnWatchdogExpired(); が呼ばれ、ここで WebSocket の再接続が行われる。
  // 現在は WebSocket がどんな理由で閉じられても、再接続するようになっている
  ReconnectAfter();
}

void AyameClient::OnRead(boost::system::error_code ec,
                         std::size_t bytes_transferred,
                         std::string text) {
  RTC_LOG(LS_INFO) << __FUNCTION__ << ": " << ec;

  boost::ignore_unused(bytes_transferred);

  // 書き込みのために読み込み処理がキャンセルされた時にこのエラーになるので、これはエラーとして扱わない
  if (ec == boost::asio::error::operation_aborted)
    return;

  // WebSocket が closed なエラーが返ってきた場合すぐに Close(); を呼んで、OnRead 関数から抜ける
  if (ec == boost::beast::websocket::error::closed) {
    // Close(); で WebSocket が閉じられたら、OnClose(); -> ReconnectAfter(); -> OnWatchdogExpired(); の順に関数が呼ばれることで、
    // WebSocket の再接続が行われる
    Close();
    return;
  }

  if (ec)
    return MOMO_BOOST_ERROR(ec, "Read");

  RTC_LOG(LS_INFO) << __FUNCTION__ << ": text=" << text;

  auto json_message = boost::json::parse(text);
  const std::string type = json_message.at("type").as_string().c_str();
  if (type == "accept") {
    SetIceServersFromConfig(json_message);
    CreatePeerConnection();
    // isExistUser フラグが存在するか確認する
    auto is_exist_user = false;
    if (json_message.as_object().count("isExistUser") != 0) {
      has_is_exist_user_flag_ = true;
      is_exist_user = json_message.at("isExistUser").as_bool();
    }

    auto on_create_offer = [this](webrtc::SessionDescriptionInterface* desc) {
      std::string sdp;
      desc->ToString(&sdp);
      manager_->SetParameters();
      boost::json::value json_message = {{"type", "offer"}, {"sdp", sdp}};
      ws_->WriteText(boost::json::serialize(json_message));
    };

    // isExistUser フラグが存在してかつ true な場合 offer SDP を生成して送信する
    if (is_exist_user) {
      RTC_LOG(LS_INFO) << __FUNCTION__ << ": exist_user";
      is_send_offer_ = true;
      connection_->CreateOffer(on_create_offer);
    } else if (!has_is_exist_user_flag_) {
      // フラグがない場合とりあえず送信
      connection_->CreateOffer(on_create_offer);
    }
  } else if (type == "offer") {
    // isExistUser フラグがなかった場合二回 peer connection を生成する
    if (!has_is_exist_user_flag_) {
      CreatePeerConnection();
    }
    const std::string sdp = json_message.at("sdp").as_string().c_str();
    connection_->SetOffer(sdp, [this]() {
      boost::asio::post(ioc_, [this, self = shared_from_this()]() {
        if (!is_send_offer_ || !has_is_exist_user_flag_) {
          connection_->CreateAnswer(
              [this](webrtc::SessionDescriptionInterface* desc) {
                std::string sdp;
                desc->ToString(&sdp);
                manager_->SetParameters();
                boost::json::value json_message = {{"type", "answer"},
                                                   {"sdp", sdp}};
                ws_->WriteText(boost::json::serialize(json_message));
              });
        }
        is_send_offer_ = false;
      });
    });
  } else if (type == "answer") {
    const std::string sdp = json_message.at("sdp").as_string().c_str();
    connection_->SetAnswer(sdp);
  } else if (type == "candidate") {
    boost::json::value ice = json_message.at("ice");
    std::string sdp_mid = ice.at("sdpMid").as_string().c_str();
    int sdp_mlineindex = ice.at("sdpMLineIndex").to_number<int>();
    std::string candidate = ice.at("candidate").as_string().c_str();
    connection_->AddIceCandidate(sdp_mid, sdp_mlineindex, candidate);
  } else if (type == "ping") {
    watchdog_.Reset();
    DoSendPong();
  } else if (type == "bye") {
    RTC_LOG(LS_INFO) << __FUNCTION__ << ": bye";
    connection_ = nullptr;
    Close();
  }
  DoRead();
}

// WebRTC からのコールバック
// これらは別スレッドからやってくるので取り扱い注意
void AyameClient::OnIceConnectionStateChange(
    webrtc::PeerConnectionInterface::IceConnectionState new_state) {
  RTC_LOG(LS_INFO) << __FUNCTION__ << " state:" << new_state;
  // デストラクタだと shared_from_this が機能しないので無視する
  if (destructed_) {
    return;
  }
  boost::asio::post(ioc_, std::bind(&AyameClient::DoIceConnectionStateChange,
                                    shared_from_this(), new_state));
}
void AyameClient::OnIceCandidate(const std::string sdp_mid,
                                 const int sdp_mlineindex,
                                 const std::string sdp) {
  // ayame では candidate sdp の交換で `ice` プロパティを用いる。 `candidate` ではないので注意
  boost::json::value json_message = {
      {"type", "candidate"},
  };
  // ice プロパティの中に object で candidate 情報をセットして送信する
  json_message.as_object()["ice"] = {{"candidate", sdp},
                                     {"sdpMLineIndex", sdp_mlineindex},
                                     {"sdpMid", sdp_mid}};
  ws_->WriteText(boost::json::serialize(json_message));
}

void AyameClient::DoIceConnectionStateChange(
    webrtc::PeerConnectionInterface::IceConnectionState new_state) {
  RTC_LOG(LS_INFO) << __FUNCTION__ << ": newState="
                   << Util::IceConnectionStateToString(new_state);

  switch (new_state) {
    case webrtc::PeerConnectionInterface::IceConnectionState::
        kIceConnectionConnected:
      retry_count_ = 0;
      watchdog_.Enable(60);
      break;
    // ice connection state が failed になったら Close(); を呼んで、WebSocket 接続を閉じる
    case webrtc::PeerConnectionInterface::IceConnectionState::
        kIceConnectionFailed:
      // Close(); で WebSocket が閉じられたら、OnClose(); -> ReconnectAfter(); -> OnWatchdogExpired(); の順に関数が呼ばれることで
      // WebSocket の再接続が行われる
      Close();
      break;
    default:
      break;
  }
  rtc_state_ = new_state;
}
