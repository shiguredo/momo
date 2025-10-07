#include "ayame_client.h"

// boost
#include <boost/beast/websocket/stream.hpp>
#include <boost/json.hpp>

// WebRTC
#include <api/peer_connection_interface.h>
#include <api/rtp_transceiver_interface.h>

#include <algorithm>
#include <cctype>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "momo_version.h"
#include "ssl_verifier.h"
#include "url_parts.h"
#include "util.h"

namespace {
// 映像補助コーデック
const std::vector<std::string> kVideoAuxiliaryCodecs = {"rtx", "red", "ulpfec",
                                                        "flexfec-03"};

// 音声補助コーデック
const std::vector<std::string> kAudioAuxiliaryCodecs = {"telephone-event",
                                                        "cn"};

std::string NormalizeCodecName(const std::string& codec_name) {
  std::string normalized = codec_name;
  std::transform(
      normalized.begin(), normalized.end(), normalized.begin(), [](char c) {
        return static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
      });
  return normalized;
}

constexpr int kInitialWatchdogTimeoutSeconds = 30;
constexpr int kConnectedWatchdogTimeoutSeconds = 60;
constexpr int kReconnectIntervalStepSeconds = 10;
constexpr int kReconnectIntervalMaxSeconds = 30;

// コーデックが補助コーデックかどうかを判定
bool IsAuxiliaryCodec(const std::string& codec_name,
                      webrtc::MediaType media_type) {
  const auto& auxiliary_codecs = media_type == webrtc::MediaType::VIDEO
                                     ? kVideoAuxiliaryCodecs
                                     : kAudioAuxiliaryCodecs;
  const std::string normalized = NormalizeCodecName(codec_name);
  return std::find(auxiliary_codecs.begin(), auxiliary_codecs.end(),
                   normalized) != auxiliary_codecs.end();
}
}  // namespace

bool AyameClient::ParseURL(URLParts& parts) const {
  const std::string& url = config_.signaling_url;

  if (!URLParts::Parse(url, parts)) {
    throw std::runtime_error("failed to parse signaling url: " + url);
  }

  if (parts.scheme == "wss") {
    return true;
  } else if (parts.scheme == "ws") {
    return false;
  } else {
    throw std::runtime_error("unsupported signaling scheme: " + parts.scheme);
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

  // WebSocket を作り直す前に明示的に破棄してから URL を検証する
  ws_.reset();

  URLParts parts;
  bool use_tls = false;
  try {
    use_tls = ParseURL(parts);
  } catch (const std::exception& e) {
    RTC_LOG(LS_ERROR) << __FUNCTION__
                      << ": failed to prepare signaling url: " << e.what();
    throw;
  }

  if (use_tls) {
    ws_ = std::make_unique<Websocket>(Websocket::ssl_tag(), ioc_,
                                      config_.insecure, config_.client_cert,
                                      config_.client_key);
  } else {
    ws_ = std::make_unique<Websocket>(ioc_);
  }
}

void AyameClient::Connect() {
  RTC_LOG(LS_INFO) << __FUNCTION__;

  watchdog_.Enable(kInitialWatchdogTimeoutSeconds);

  ws_->Connect(config_.signaling_url,
               std::bind(&AyameClient::OnConnect, shared_from_this(),
                         std::placeholders::_1));
}

void AyameClient::ReconnectAfter() {
  // retry_count_ に応じて遅延時間を伸ばしつつ、上限とオーバーフローを避ける
  const int64_t interval_raw =
      static_cast<int64_t>(retry_count_) * kReconnectIntervalStepSeconds;
  const int64_t clamped_interval =
      std::min<int64_t>(interval_raw, kReconnectIntervalMaxSeconds);
  const int interval = static_cast<int>(clamped_interval);

  RTC_LOG(LS_INFO) << __FUNCTION__ << " reconnect after " << interval << " sec";

  watchdog_.Enable(interval);
  if (retry_count_ < std::numeric_limits<int>::max()) {
    retry_count_++;
  }
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
    MOMO_BOOST_ERROR(ec, "Handshake");
    return;
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
      for (const auto& jserver : jservers.as_array()) {
        const auto* jserver_obj = jserver.if_object();
        if (jserver_obj == nullptr) {
          RTC_LOG(LS_WARNING)
              << __FUNCTION__ << ": skip invalid iceServers entry";
          continue;
        }

        webrtc::PeerConnectionInterface::IceServer ice_server;
        if (const auto* username = jserver_obj->if_contains("username");
            username != nullptr && username->is_string()) {
          ice_server.username = username->as_string().c_str();
        }
        if (const auto* credential = jserver_obj->if_contains("credential");
            credential != nullptr && credential->is_string()) {
          ice_server.password = credential->as_string().c_str();
        }

        const auto* urls_value = jserver_obj->if_contains("urls");
        if (urls_value == nullptr) {
          RTC_LOG(LS_WARNING)
              << __FUNCTION__ << ": iceServers entry has no urls field";
          continue;
        }

        auto add_url = [&](const boost::json::string& json_url) {
          std::string url_text = json_url.c_str();
          ice_server.urls.push_back(url_text);
          RTC_LOG(LS_INFO) << __FUNCTION__ << ": iceserver.url=" << url_text;
        };

        if (urls_value->is_array()) {
          for (const auto& url : urls_value->as_array()) {
            if (!url.is_string()) {
              RTC_LOG(LS_WARNING)
                  << __FUNCTION__
                  << ": skip non string url entry in iceServers";
              continue;
            }
            add_url(url.as_string());
          }
        } else if (urls_value->is_string()) {
          add_url(urls_value->as_string());
        } else {
          RTC_LOG(LS_WARNING)
              << __FUNCTION__
              << ": urls field must be string or array of strings";
        }

        if (ice_server.urls.empty()) {
          continue;
        }

        ice_servers_.push_back(ice_server);
      }
    }
  }
  if (ice_servers_.empty() && !config_.no_google_stun) {
    // accept 時に iceServers が返却されてこなかった場合 google の stun server を追加する
    webrtc::PeerConnectionInterface::IceServer ice_server;
    ice_server.urls.push_back("stun:stun.l.google.com:19302");
    ice_servers_.push_back(ice_server);
  }
}

bool AyameClient::CreatePeerConnection() {
  webrtc::PeerConnectionInterface::RTCConfiguration rtc_config;

  rtc_config.servers = ice_servers_;
  connection_ = manager_->CreateConnection(rtc_config, this);
  if (!connection_) {
    RTC_LOG(LS_ERROR) << __FUNCTION__ << ": failed to create RTC connection";
    return false;
  }
  manager_->InitTracks(connection_.get(), config_.direction);

  // InitTracks で Transceiver が作成された後に SetCodecPreferences を呼ぶ
  SetCodecPreferences();
  return true;
}

void AyameClient::SetCodecPreferences() {
  if (config_.video_codec_type.empty() && config_.audio_codec_type.empty()) {
    return;
  }

  if (!connection_) {
    RTC_LOG(LS_ERROR) << "RTC connection is null";
    return;
  }

  auto pc = connection_->GetConnection();
  if (pc == nullptr) {
    RTC_LOG(LS_ERROR) << "PeerConnection is null";
    return;
  }

  // PeerConnectionFactory から GetRtpSenderCapabilities を使ってコーデック一覧を取得
  auto factory = manager_->GetFactory();
  if (factory == nullptr) {
    RTC_LOG(LS_ERROR) << "PeerConnectionFactory is null";
    return;
  }

  auto transceivers = pc->GetTransceivers();

  // Transceiver が存在しない場合はエラーを出して何もしない
  if (transceivers.empty()) {
    RTC_LOG(LS_ERROR)
        << "No transceivers found when trying to set codec preferences";
    return;
  }

  for (auto transceiver : transceivers) {
    const auto media_type = transceiver->media_type();
    const bool is_video = media_type == webrtc::MediaType::VIDEO;
    const bool is_audio = media_type == webrtc::MediaType::AUDIO;

    // VIDEO でも AUDIO でもない Transceiver はスキップ
    if (!is_video && !is_audio) {
      continue;
    }

    // 指定されたコーデックが空の場合はスキップ
    const bool codec_not_specified =
        (is_video && config_.video_codec_type.empty()) ||
        (is_audio && config_.audio_codec_type.empty());
    if (codec_not_specified) {
      continue;
    }

    const std::string& target_codec =
        is_video ? config_.video_codec_type : config_.audio_codec_type;
    const std::string normalized_target = NormalizeCodecName(target_codec);

    // PeerConnectionFactory から送信側と受信側の両方の capabilities を取得
    webrtc::RtpCapabilities sender_capabilities =
        factory->GetRtpSenderCapabilities(transceiver->media_type());
    webrtc::RtpCapabilities receiver_capabilities =
        factory->GetRtpReceiverCapabilities(transceiver->media_type());

    // 送信側と受信側の両方でサポートされているコーデックを見つける
    std::vector<webrtc::RtpCodecCapability> common_codecs;
    for (const auto& sender_codec : sender_capabilities.codecs) {
      for (const auto& receiver_codec : receiver_capabilities.codecs) {
        // MIMEタイプが一致する場合に共通コーデックとみなす
        if (sender_codec.mime_type() == receiver_codec.mime_type()) {
          common_codecs.push_back(sender_codec);
          break;
        }
      }
    }

    // 共通コーデックが空の場合はスキップ
    if (common_codecs.empty()) {
      RTC_LOG(LS_WARNING)
          << "No common codec capabilities available for transceiver";
      continue;
    }

    RTC_LOG(LS_INFO) << "Found " << common_codecs.size()
                     << " common codecs for "
                     << webrtc::MediaTypeToString(media_type);

    // コーデックのフィルタリング
    std::vector<webrtc::RtpCodecCapability> primary_codecs;
    std::vector<webrtc::RtpCodecCapability> auxiliary_codecs;
    for (const auto& codec : common_codecs) {
      if (NormalizeCodecName(codec.name) == normalized_target) {
        primary_codecs.push_back(codec);
        continue;
      }
      if (IsAuxiliaryCodec(codec.name, media_type)) {
        auxiliary_codecs.push_back(codec);
      }
    }

    // 指定されたコーデックが見つからなかった場合はエラー
    if (primary_codecs.empty()) {
      RTC_LOG(LS_ERROR) << "Specified codec '" << target_codec << "' for "
                        << webrtc::MediaTypeToString(transceiver->media_type())
                        << " is not available. Available codecs:";
      for (const auto& codec : common_codecs) {
        RTC_LOG(LS_ERROR) << "  - " << codec.name;
      }
      continue;
    }

    std::vector<webrtc::RtpCodecCapability> filtered_codecs;
    filtered_codecs.reserve(primary_codecs.size() + auxiliary_codecs.size());
    filtered_codecs.insert(filtered_codecs.end(), primary_codecs.begin(),
                           primary_codecs.end());
    filtered_codecs.insert(filtered_codecs.end(), auxiliary_codecs.begin(),
                           auxiliary_codecs.end());

    auto error = transceiver->SetCodecPreferences(filtered_codecs);
    if (!error.ok()) {
      RTC_LOG(LS_ERROR) << "Failed to set codec preferences: "
                        << error.message();
      continue;
    }

    RTC_LOG(LS_INFO) << "Successfully set codec preferences for "
                     << webrtc::MediaTypeToString(transceiver->media_type());
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
                         [[maybe_unused]] std::size_t bytes_transferred,
                         std::string text) {
  RTC_LOG(LS_INFO) << __FUNCTION__ << ": " << ec;

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

  if (ec) {
    MOMO_BOOST_ERROR(ec, "Read");
    return;
  }

  RTC_LOG(LS_INFO) << __FUNCTION__ << ": text=" << text;

  auto json_message = boost::json::parse(text);
  const std::string type = json_message.at("type").as_string().c_str();
  if (type == "accept") {
    SetIceServersFromConfig(json_message);
    if (!CreatePeerConnection()) {
      RTC_LOG(LS_ERROR) << __FUNCTION__
                        << ": peer connection setup failed at accept";
      Close();
      return;
    }
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
      if (!CreatePeerConnection()) {
        RTC_LOG(LS_ERROR) << __FUNCTION__
                          << ": peer connection setup failed at offer";
        Close();
        return;
      }
    }
    if (!connection_) {
      RTC_LOG(LS_ERROR) << __FUNCTION__
                        << ": peer connection is not ready for offer";
      Close();
      return;
    }
    const std::string sdp = json_message.at("sdp").as_string().c_str();
    auto self = shared_from_this();
    connection_->SetOffer(sdp, [self]() {
      auto& client = *self;
      boost::asio::post(client.ioc_, [self]() {
        auto& client = *self;
        // Answer を作成する条件:
        // 1. 自分から Offer を送信していない場合 (!is_send_offer_)
        // 2. isExistUser フラグがなかった場合 (!has_is_exist_user_flag_)
        // isExistUser フラグがある場合は既存ユーザーがいることを示すため、
        // 2回目の Offer を受信した時にのみ Answer を作成する
        const bool should_create_answer =
            !client.is_send_offer_ || !client.has_is_exist_user_flag_;
        if (should_create_answer) {
          client.connection_->CreateAnswer(
              [self](webrtc::SessionDescriptionInterface* desc) {
                auto& client = *self;
                std::string sdp;
                desc->ToString(&sdp);
                client.manager_->SetParameters();
                boost::json::value json_message = {{"type", "answer"},
                                                   {"sdp", sdp}};
                client.ws_->WriteText(boost::json::serialize(json_message));
              });
        }
        client.is_send_offer_ = false;
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
      watchdog_.Enable(kConnectedWatchdogTimeoutSeconds);
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
