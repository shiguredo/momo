#include "sora_client.h"

#include <fstream>
#include <sstream>

// boost
#include <boost/beast/websocket/stream.hpp>

// json
#include <nlohmann/json.hpp>

#include "momo_version.h"
#include "ssl_verifier.h"
#include "url_parts.h"
#include "util.h"

using json = nlohmann::json;

bool SoraClient::ParseURL(URLParts& parts) const {
  std::string url = config_.signaling_host;

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

  ws_->Connect(config_.signaling_host,
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
  json json_message = {
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

  if (config_.spotlight > 0) {
    json_message["multistream"] = true;
    json_message["spotlight"] = config_.spotlight;
  }

  if (!config_.metadata.is_null()) {
    json_message["metadata"] = config_.metadata;
  }

  if (!config_.video) {
    // video: false の場合はそのまま設定
    json_message["video"] = false;
  } else if (config_.video && config_.video_codec.empty() &&
             config_.video_bitrate == 0) {
    // video: true の場合、その他のオプションの設定が行われてなければ true を設定
    json_message["video"] = true;
  } else {
    // それ以外はちゃんとオプションを設定する
    if (!config_.video_codec.empty()) {
      json_message["video"]["codec_type"] = config_.video_codec;
    }
    if (config_.video_bitrate != 0) {
      json_message["video"]["bit_rate"] = config_.video_bitrate;
    }
  }

  if (!config_.audio) {
    json_message["audio"] = false;
  } else if (config_.audio && config_.audio_codec.empty() &&
             config_.audio_bitrate == 0) {
    json_message["audio"] = true;
  } else {
    if (!config_.audio_codec.empty()) {
      json_message["audio"]["codec_type"] = config_.audio_codec;
    }
    if (config_.audio_bitrate != 0) {
      json_message["audio"]["bit_rate"] = config_.audio_bitrate;
    }
  }

  ws_->WriteText(json_message.dump());
}
void SoraClient::DoSendPong() {
  json json_message = {{"type", "pong"}};
  ws_->WriteText(json_message.dump());
}
void SoraClient::DoSendPong(
    const rtc::scoped_refptr<const webrtc::RTCStatsReport>& report) {
  std::string stats = report->ToJson();
  json json_message = {{"type", "pong"}, {"stats", stats}};
  std::string str = R"({"type":"pong","stats":)" + stats + "}";
  ws_->WriteText(std::move(str));
}

void SoraClient::CreatePeerFromConfig(json jconfig) {
  webrtc::PeerConnectionInterface::RTCConfiguration rtc_config;
  webrtc::PeerConnectionInterface::IceServers ice_servers;

  auto jservers = jconfig["iceServers"];
  for (auto jserver : jservers) {
    const std::string username = jserver["username"];
    const std::string credential = jserver["credential"];
    auto jurls = jserver["urls"];
    for (const std::string url : jurls) {
      webrtc::PeerConnectionInterface::IceServer ice_server;
      ice_server.uri = url;
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

  auto json_message = json::parse(text);
  const std::string type = json_message["type"];
  if (type == "offer") {
    CreatePeerFromConfig(json_message["config"]);
    const std::string sdp = json_message["sdp"].get<std::string>();

    connection_->SetOffer(sdp, [this, json_message]() {
      // simulcast では offer の setRemoteDescription が終わった後に
      // トラックを追加する必要があるため、ここで初期化する
      manager_->InitTracks(connection_.get());

      if (config_.simulcast) {
        std::vector<webrtc::RtpEncodingParameters> encoding_parameters;

        // "encodings" キーの各内容を webrtc::RtpEncodingParameters に変換する
        auto encodings_json = json_message["encodings"];
        for (auto p : encodings_json) {
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
          params.rid = p["rid"].get<std::string>();
          if (p.contains("maxBitrate")) {
            params.max_bitrate_bps = p["maxBitrate"].get<int>();
          }
          if (p.contains("minBitrate")) {
            params.min_bitrate_bps = p["minBitrate"].get<int>();
          }
          if (p.contains("scaleResolutionDownBy")) {
            params.scale_resolution_down_by =
                p["scaleResolutionDownBy"].get<double>();
          }
          encoding_parameters.push_back(params);
        }
        connection_->SetEncodingParameters(std::move(encoding_parameters));
      }

      connection_->CreateAnswer(
          [this](webrtc::SessionDescriptionInterface* desc) {
            std::string sdp;
            desc->ToString(&sdp);
            json json_message = {{"type", "answer"}, {"sdp", sdp}};
            ws_->WriteText(json_message.dump());
          });
    });
  } else if (type == "update") {
    const std::string sdp = json_message["sdp"].get<std::string>();
    connection_->SetOffer(sdp, [this]() {
      connection_->CreateAnswer(
          [this](webrtc::SessionDescriptionInterface* desc) {
            std::string sdp;
            desc->ToString(&sdp);
            json json_message = {{"type", "update"}, {"sdp", sdp}};
            ws_->WriteText(json_message.dump());
          });
    });
  } else if (type == "notify") {
    const std::string event_type =
        json_message["event_type"].get<std::string>();
    if (event_type == "connection.created" ||
        event_type == "connection.destroyed") {
      RTC_LOG(LS_INFO) << __FUNCTION__ << ": event_type=" << event_type
                       << ": client_id=" << json_message["client_id"]
                       << ": connection_id=" << json_message["connection_id"];
    } else if (event_type == "network.status") {
      RTC_LOG(LS_INFO) << __FUNCTION__ << ": event_type=" << event_type
                       << ": unstable_level=" << json_message["unstable_level"];
    } else if (event_type == "spotlight.changed") {
      RTC_LOG(LS_INFO) << __FUNCTION__ << ": event_type=" << event_type
                       << ": client_id=" << json_message["client_id"]
                       << ": connection_id=" << json_message["connection_id"]
                       << ": spotlight_id=" << json_message["spotlight_id"];
    }
  } else if (type == "ping") {
    if (rtc_state_ != webrtc::PeerConnectionInterface::IceConnectionState::
                          kIceConnectionConnected) {
      DoRead();
      return;
    }
    watchdog_.Reset();
    bool stats = json_message.value("stats", false);
    if (stats) {
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
  json json_message = {{"type", "candidate"}, {"candidate", sdp}};
  ws_->WriteText(json_message.dump());
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
