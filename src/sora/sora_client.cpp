#include "sora_client.h"

#include <fstream>
#include <sstream>

// boost
#include <boost/beast/websocket/stream.hpp>
#include <boost/json.hpp>

#include "momo_version.h"
#include "ssl_verifier.h"
#include "url_parts.h"
#include "util.h"

bool SoraClient::ParseURL(URLParts& parts) const {
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

webrtc::PeerConnectionInterface::IceConnectionState
SoraClient::GetRTCConnectionState() const {
  return rtc_state_;
}

std::shared_ptr<RTCConnection> SoraClient::GetRTCConnection() const {
  if (rtc_state_ == webrtc::PeerConnectionInterface::IceConnectionState::
                        kIceConnectionConnected) {
    return connection_;
  } else {
    return nullptr;
  }
}

void SoraClient::GetStats(
    std::function<void(const rtc::scoped_refptr<const webrtc::RTCStatsReport>&)>
        callback) {
  std::shared_ptr<RTCConnection> rtc_conn = GetRTCConnection();
  if (rtc_conn) {
    rtc_conn->GetStats(std::move(callback));
  } else {
    callback(nullptr);
  }
}

SoraClient::SoraClient(boost::asio::io_context& ioc,
                       RTCManager* manager,
                       SoraClientConfig config)
    : ioc_(ioc),
      manager_(manager),
      retry_count_(0),
      config_(std::move(config)),
      watchdog_(ioc, std::bind(&SoraClient::OnWatchdogExpired, this)) {
  Reset();
}

SoraClient::~SoraClient() {
  destructed_ = true;
  // ここで OnIceConnectionStateChange が呼ばれる
  connection_ = nullptr;
}

void SoraClient::Reset() {
  connection_ = nullptr;

  URLParts parts;
  if (ParseURL(parts)) {
    ws_.reset(new Websocket(Websocket::ssl_tag(), ioc_, config_.insecure));
  } else {
    ws_.reset(new Websocket(ioc_));
  }
}

void SoraClient::Connect() {
  RTC_LOG(LS_INFO) << __FUNCTION__;

  watchdog_.Enable(30);

  ws_->Connect(config_.signaling_url,
               std::bind(&SoraClient::OnConnect, shared_from_this(),
                         std::placeholders::_1));
}

void SoraClient::ReconnectAfter() {
  int interval = 5 * (2 * retry_count_ + 1);
  RTC_LOG(LS_INFO) << __FUNCTION__ << " reconnect after " << interval << " sec";

  watchdog_.Enable(interval);
  retry_count_++;
}

void SoraClient::OnWatchdogExpired() {
  RTC_LOG(LS_INFO) << __FUNCTION__ << " reconnecting...:";
  Reset();
  Connect();
}

void SoraClient::OnConnect(boost::system::error_code ec) {
  if (ec) {
    ReconnectAfter();
    return MOMO_BOOST_ERROR(ec, "Handshake");
  }

  DoRead();
  DoSendConnect();
}
void SoraClient::DoRead() {
  ws_->Read(std::bind(&SoraClient::OnRead, shared_from_this(),
                      std::placeholders::_1, std::placeholders::_2,
                      std::placeholders::_3));
}

void SoraClient::DoSendConnect() {
  boost::json::object json_message = {
      {"type", "connect"},
      {"role", config_.role},
      {"channel_id", config_.channel_id},
      {"sora_client", MomoVersion::GetClientName()},
      {"libwebrtc", MomoVersion::GetLibwebrtcName()},
      {"environment", MomoVersion::GetEnvironmentName()},
  };

  if (config_.multistream) {
    json_message["multistream"] = true;
  }

  if (config_.simulcast) {
    json_message["simulcast"] = true;
  }

  if (config_.spotlight) {
    json_message["spotlight"] = true;
  }
  if (config_.spotlight && config_.spotlight_number > 0) {
    json_message["spotlight_number"] = config_.spotlight_number;
  }

  if (!config_.metadata.is_null()) {
    json_message["metadata"] = config_.metadata;
  }

  if (!config_.video) {
    // video: false の場合はそのまま設定
    json_message["video"] = false;
  } else if (config_.video && config_.video_codec_type.empty() &&
             config_.video_bit_rate == 0) {
    // video: true の場合、その他のオプションの設定が行われてなければ true を設定
    json_message["video"] = true;
  } else {
    // それ以外はちゃんとオプションを設定する
    json_message["video"] = boost::json::object();
    if (!config_.video_codec_type.empty()) {
      json_message["video"].as_object()["codec_type"] =
          config_.video_codec_type;
    }
    if (config_.video_bit_rate != 0) {
      json_message["video"].as_object()["bit_rate"] = config_.video_bit_rate;
    }
  }

  if (!config_.audio) {
    json_message["audio"] = false;
  } else if (config_.audio && config_.audio_codec_type.empty() &&
             config_.audio_bit_rate == 0) {
    json_message["audio"] = true;
  } else {
    json_message["audio"] = boost::json::object();
    if (!config_.audio_codec_type.empty()) {
      json_message["audio"].as_object()["codec_type"] =
          config_.audio_codec_type;
    }
    if (config_.audio_bit_rate != 0) {
      json_message["audio"].as_object()["bit_rate"] = config_.audio_bit_rate;
    }
  }

  ws_->WriteText(boost::json::serialize(json_message));
}
void SoraClient::DoSendPong() {
  boost::json::value json_message = {{"type", "pong"}};
  ws_->WriteText(boost::json::serialize(json_message));
}
void SoraClient::DoSendPong(
    const rtc::scoped_refptr<const webrtc::RTCStatsReport>& report) {
  std::string stats = report->ToJson();
  boost::json::value json_message = {{"type", "pong"}, {"stats", stats}};
  std::string str = R"({"type":"pong","stats":)" + stats + "}";
  ws_->WriteText(std::move(str));
}

void SoraClient::CreatePeerFromConfig(boost::json::value jconfig) {
  webrtc::PeerConnectionInterface::RTCConfiguration rtc_config;
  webrtc::PeerConnectionInterface::IceServers ice_servers;

  auto jservers = jconfig.at("iceServers");
  for (auto jserver : jservers.as_array()) {
    const std::string username = jserver.at("username").as_string().c_str();
    const std::string credential = jserver.at("credential").as_string().c_str();
    auto jurls = jserver.at("urls");
    for (const auto url : jurls.as_array()) {
      webrtc::PeerConnectionInterface::IceServer ice_server;
      ice_server.uri = url.as_string().c_str();
      ice_server.username = username;
      ice_server.password = credential;
      ice_servers.push_back(ice_server);
    }
  }

  rtc_config.servers = ice_servers;

  // macOS のサイマルキャスト時、なぜか無限に解像度が落ちていくので、
  // それを回避するために cpu_adaptation を無効にする。
#if defined(__APPLE__)
  if (config_.simulcast) {
    rtc_config.set_cpu_adaptation(false);
  }
#endif

  connection_ = manager_->CreateConnection(rtc_config, this);
}

void SoraClient::Close() {
  ws_->Close(std::bind(&SoraClient::OnClose, shared_from_this(),
                       std::placeholders::_1));
}

void SoraClient::OnClose(boost::system::error_code ec) {
  if (ec)
    return MOMO_BOOST_ERROR(ec, "close");
}

void SoraClient::OnRead(boost::system::error_code ec,
                        std::size_t bytes_transferred,
                        std::string text) {
  RTC_LOG(LS_INFO) << __FUNCTION__ << ": " << ec;

  boost::ignore_unused(bytes_transferred);

  // 書き込みのために読み込み処理がキャンセルされた時にこのエラーになるので、これはエラーとして扱わない
  if (ec == boost::asio::error::operation_aborted)
    return;

  if (ec)
    return MOMO_BOOST_ERROR(ec, "Read");

  RTC_LOG(LS_INFO) << __FUNCTION__ << ": text=" << text;

  auto json_message = boost::json::parse(text);
  const std::string type = json_message.at("type").as_string().c_str();
  if (type == "offer") {
    CreatePeerFromConfig(json_message.at("config"));
    const std::string sdp = json_message.at("sdp").as_string().c_str();

    connection_->SetOffer(sdp, [this, json_message]() {
      // simulcast では offer の setRemoteDescription が終わった後に
      // トラックを追加する必要があるため、ここで初期化する
      manager_->InitTracks(connection_.get());

      if (config_.simulcast &&
          json_message.as_object().count("encodings") != 0) {
        std::vector<webrtc::RtpEncodingParameters> encoding_parameters;

        // "encodings" キーの各内容を webrtc::RtpEncodingParameters に変換する
        auto encodings_json = json_message.at("encodings").as_array();
        for (auto v : encodings_json) {
          auto p = v.as_object();
          webrtc::RtpEncodingParameters params;
          // absl::optional<uint32_t> ssrc;
          // double bitrate_priority = kDefaultBitratePriority;
          // enum class Priority { kVeryLow, kLow, kMedium, kHigh };
          // Priority network_priority = Priority::kLow;
          // absl::optional<int> max_bitrate_bps;
          // absl::optional<int> min_bitrate_bps;
          // absl::optional<double> max_framerate;
          // absl::optional<int> num_temporal_layers;
          // absl::optional<double> scale_resolution_down_by;
          // bool active = true;
          // std::string rid;
          // bool adaptive_ptime = false;
          params.rid = p["rid"].as_string().c_str();
          if (p.count("maxBitrate") != 0) {
            params.max_bitrate_bps = p["maxBitrate"].to_number<int>();
          }
          if (p.count("minBitrate") != 0) {
            params.min_bitrate_bps = p["minBitrate"].to_number<int>();
          }
          if (p.count("scaleResolutionDownBy") != 0) {
            params.scale_resolution_down_by =
                p["scaleResolutionDownBy"].to_number<double>();
          }
          if (p.count("maxFramerate") != 0) {
            params.max_framerate = p["maxFramerate"].to_number<double>();
          }
          if (p.count("active") != 0) {
            params.active = p["active"].as_bool();
          }
          if (p.count("adaptivePtime") != 0) {
            params.adaptive_ptime = p["adaptivePtime"].as_bool();
          }
          encoding_parameters.push_back(params);
        }
        connection_->SetEncodingParameters(std::move(encoding_parameters));
      }

      connection_->CreateAnswer(
          [this](webrtc::SessionDescriptionInterface* desc) {
            std::string sdp;
            desc->ToString(&sdp);
            boost::json::value json_message = {{"type", "answer"},
                                               {"sdp", sdp}};
            ws_->WriteText(boost::json::serialize(json_message));
          });
    });
  } else if (type == "update") {
    const std::string sdp = json_message.at("sdp").as_string().c_str();
    connection_->SetOffer(sdp, [this]() {
      connection_->CreateAnswer(
          [this](webrtc::SessionDescriptionInterface* desc) {
            std::string sdp;
            desc->ToString(&sdp);
            boost::json::value json_message = {{"type", "update"},
                                               {"sdp", sdp}};
            ws_->WriteText(boost::json::serialize(json_message));
          });
    });
  } else if (type == "notify") {
    const std::string event_type =
        json_message.at("event_type").as_string().c_str();
    if (event_type == "connection.created" ||
        event_type == "connection.destroyed") {
      RTC_LOG(LS_INFO) << __FUNCTION__ << ": event_type=" << event_type
                       << ": client_id=" << json_message.at("client_id")
                       << ": connection_id="
                       << json_message.at("connection_id");
    } else if (event_type == "network.status") {
      RTC_LOG(LS_INFO) << __FUNCTION__ << ": event_type=" << event_type
                       << ": unstable_level="
                       << json_message.at("unstable_level");
    } else if (event_type == "spotlight.changed") {
      RTC_LOG(LS_INFO) << __FUNCTION__ << ": event_type=" << event_type
                       << ": client_id=" << json_message.at("client_id")
                       << ": connection_id=" << json_message.at("connection_id")
                       << ": spotlight_id=" << json_message.at("spotlight_id");
    }
  } else if (type == "ping") {
    if (rtc_state_ != webrtc::PeerConnectionInterface::IceConnectionState::
                          kIceConnectionConnected) {
      DoRead();
      return;
    }
    watchdog_.Reset();
    auto it = json_message.as_object().find("stats");
    if (it != json_message.as_object().end() && it->value().as_bool()) {
      connection_->GetStats(
          [this](
              const rtc::scoped_refptr<const webrtc::RTCStatsReport>& report) {
            DoSendPong(report);
          });
    } else {
      DoSendPong();
    }
  }
  DoRead();
}

// WebRTC からのコールバック
// これらは別スレッドからやってくるので取り扱い注意
void SoraClient::OnIceConnectionStateChange(
    webrtc::PeerConnectionInterface::IceConnectionState new_state) {
  RTC_LOG(LS_INFO) << __FUNCTION__ << " state:" << new_state;
  // デストラクタだと shared_from_this が機能しないので無視する
  if (destructed_) {
    return;
  }
  boost::asio::post(ioc_, std::bind(&SoraClient::DoIceConnectionStateChange,
                                    shared_from_this(), new_state));
}
void SoraClient::OnIceCandidate(const std::string sdp_mid,
                                const int sdp_mlineindex,
                                const std::string sdp) {
  boost::json::value json_message = {{"type", "candidate"}, {"candidate", sdp}};
  ws_->WriteText(boost::json::serialize(json_message));
}

void SoraClient::DoIceConnectionStateChange(
    webrtc::PeerConnectionInterface::IceConnectionState new_state) {
  RTC_LOG(LS_INFO) << __FUNCTION__ << ": newState="
                   << Util::IceConnectionStateToString(new_state);

  switch (new_state) {
    case webrtc::PeerConnectionInterface::IceConnectionState::
        kIceConnectionConnected:
      retry_count_ = 0;
      watchdog_.Enable(60);
      break;
    case webrtc::PeerConnectionInterface::IceConnectionState::
        kIceConnectionFailed:
      ReconnectAfter();
      break;
    default:
      break;
  }
  rtc_state_ = new_state;
}
