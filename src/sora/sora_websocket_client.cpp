#include "sora_websocket_client.h"

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

bool SoraWebsocketClient::parseURL(URLParts& parts) const {
  std::string url = conn_settings_.sora_signaling_host;

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

boost::asio::ssl::context SoraWebsocketClient::createSSLContext() const {
  boost::asio::ssl::context ctx(boost::asio::ssl::context::tlsv12);
  //ctx.set_default_verify_paths();
  ctx.set_options(boost::asio::ssl::context::default_workarounds |
                  boost::asio::ssl::context::no_sslv2 |
                  boost::asio::ssl::context::no_sslv3 |
                  boost::asio::ssl::context::single_dh_use);
  return ctx;
}

webrtc::PeerConnectionInterface::IceConnectionState
SoraWebsocketClient::getRTCConnectionState() const {
  return rtc_state_;
}

std::shared_ptr<RTCConnection> SoraWebsocketClient::getRTCConnection() const {
  if (rtc_state_ == webrtc::PeerConnectionInterface::IceConnectionState::
                        kIceConnectionConnected) {
    return connection_;
  } else {
    return nullptr;
  }
}

SoraWebsocketClient::SoraWebsocketClient(boost::asio::io_context& ioc,
                                         RTCManager* manager,
                                         ConnectionSettings conn_settings)
    : ioc_(ioc),
      resolver_(ioc),
      manager_(manager),
      retry_count_(0),
      conn_settings_(conn_settings),
      watchdog_(ioc, std::bind(&SoraWebsocketClient::onWatchdogExpired, this)) {
  reset();
}

void SoraWebsocketClient::reset() {
  connection_ = nullptr;
  connected_ = false;

  if (parseURL(parts_)) {
    auto ssl_ctx = createSSLContext();
    ws_.reset(new Websocket(ioc_, std::move(ssl_ctx)));
    ws_->NativeSecureSocket().next_layer().set_verify_mode(
        boost::asio::ssl::verify_peer);
    ws_->NativeSecureSocket().next_layer().set_verify_callback(
        [insecure = conn_settings_.insecure](
            bool preverified, boost::asio::ssl::verify_context& ctx) {
          if (preverified) {
            return true;
          }
          // insecure の場合は証明書をチェックしない
          if (insecure) {
            return true;
          }
          X509* cert = X509_STORE_CTX_get_current_cert(ctx.native_handle());
          return SSLVerifier::VerifyX509(cert);
        });

    // SNI の設定を行う
    if (!SSL_set_tlsext_host_name(
            ws_->NativeSecureSocket().next_layer().native_handle(),
            parts_.host.c_str())) {
      boost::system::error_code ec{static_cast<int>(::ERR_get_error()),
                                   boost::asio::error::get_ssl_category()};
      MOMO_BOOST_ERROR(ec, "SSL_set_tlsext_host_name");
    }
  } else {
    boost::beast::websocket::stream<boost::asio::ip::tcp::socket> ws(ioc_);
    ws_.reset(new Websocket(ioc_));
  }
}

void SoraWebsocketClient::release() {
  connection_ = nullptr;
}

bool SoraWebsocketClient::connect() {
  RTC_LOG(LS_INFO) << __FUNCTION__;

  if (connected_) {
    return false;
  }

  std::string port;
  if (parts_.port.empty()) {
    port = ws_->isSSL() ? "443" : "80";
  } else {
    port = parts_.port;
  }

  // DNS ルックアップ
  resolver_.async_resolve(
      parts_.host, port,
      boost::asio::bind_executor(
          ws_->strand(),
          std::bind(&SoraWebsocketClient::onResolve, shared_from_this(),
                    std::placeholders::_1, std::placeholders::_2)));

  watchdog_.Enable(30);

  return true;
}

void SoraWebsocketClient::reconnectAfter() {
  int interval = 5 * (2 * retry_count_ + 1);
  RTC_LOG(LS_INFO) << __FUNCTION__ << " reconnect after " << interval << " sec";

  watchdog_.Enable(interval);
  retry_count_++;
}

void SoraWebsocketClient::onWatchdogExpired() {
  RTC_LOG(LS_WARNING) << __FUNCTION__;

  RTC_LOG(LS_INFO) << __FUNCTION__ << " reconnecting...:";
  reset();
  connect();
}

void SoraWebsocketClient::onResolve(
    boost::system::error_code ec,
    boost::asio::ip::tcp::resolver::results_type results) {
  if (ec) {
    reconnectAfter();
    return MOMO_BOOST_ERROR(ec, "resolve");
  }

  // DNS ルックアップで得られたエンドポイントに対して接続する
  if (ws_->isSSL()) {
    boost::asio::async_connect(
        ws_->NativeSecureSocket().next_layer().next_layer(), results.begin(),
        results.end(),
        boost::asio::bind_executor(
            ws_->strand(),
            std::bind(&SoraWebsocketClient::onSSLConnect, shared_from_this(),
                      std::placeholders::_1)));
  } else {
    boost::asio::async_connect(
        ws_->NativeSocket().next_layer(), results.begin(), results.end(),
        boost::asio::bind_executor(
            ws_->strand(),
            std::bind(&SoraWebsocketClient::onConnect, shared_from_this(),
                      std::placeholders::_1)));
  }
}

void SoraWebsocketClient::onSSLConnect(boost::system::error_code ec) {
  if (ec) {
    reconnectAfter();
    return MOMO_BOOST_ERROR(ec, "SSLConnect");
  }

  // SSL のハンドシェイク
  ws_->NativeSecureSocket().next_layer().async_handshake(
      boost::asio::ssl::stream_base::client,
      boost::asio::bind_executor(
          ws_->strand(), std::bind(&SoraWebsocketClient::onSSLHandshake,
                                   shared_from_this(), std::placeholders::_1)));
}

void SoraWebsocketClient::onSSLHandshake(boost::system::error_code ec) {
  if (ec) {
    reconnectAfter();
    return MOMO_BOOST_ERROR(ec, "SSLHandshake");
  }

  // Websocket のハンドシェイク
  ws_->NativeSecureSocket().async_handshake(
      parts_.host, parts_.path_query_fragment,
      boost::asio::bind_executor(
          ws_->strand(), std::bind(&SoraWebsocketClient::onHandshake,
                                   shared_from_this(), std::placeholders::_1)));
}

void SoraWebsocketClient::onConnect(boost::system::error_code ec) {
  if (ec) {
    reconnectAfter();
    return MOMO_BOOST_ERROR(ec, "connect");
  }

  // Websocket のハンドシェイク
  ws_->NativeSocket().async_handshake(
      parts_.host, parts_.path_query_fragment,
      boost::asio::bind_executor(
          ws_->strand(), std::bind(&SoraWebsocketClient::onHandshake,
                                   shared_from_this(), std::placeholders::_1)));
}

void SoraWebsocketClient::onHandshake(boost::system::error_code ec) {
  if (ec) {
    reconnectAfter();
    return MOMO_BOOST_ERROR(ec, "Handshake");
  }

  connected_ = true;

  ws_->StartToRead(std::bind(&SoraWebsocketClient::onRead, this,
                             std::placeholders::_1, std::placeholders::_2,
                             std::placeholders::_3));

  doSendConnect();
}

void SoraWebsocketClient::doSendConnect() {
  const auto& cs = conn_settings_;
  json json_message = {
      {"type", "connect"},
      {"role", cs.sora_role},
      {"channel_id", cs.sora_channel_id},
      {"sora_client", MomoVersion::GetClientName()},
      {"libwebrtc", MomoVersion::GetLibwebrtcName()},
      {"environment", MomoVersion::GetEnvironmentName()},
  };

  if (cs.sora_multistream) {
    json_message["multistream"] = true;
  }

  if (cs.sora_simulcast) {
    json_message["simulcast"] = true;
  }

  if (cs.sora_spotlight > 0) {
    json_message["multistream"] = true;
    json_message["spotlight"] = cs.sora_spotlight;
  }

  if (!cs.sora_metadata.is_null()) {
    json_message["metadata"] = cs.sora_metadata;
  }

  if (!cs.sora_video) {
    // video: false の場合はそのまま設定
    json_message["video"] = false;
  } else if (cs.sora_video && cs.sora_video_codec.empty() &&
             cs.sora_video_bitrate == 0) {
    // video: true の場合、その他のオプションの設定が行われてなければ true を設定
    json_message["video"] = true;
  } else {
    // それ以外はちゃんとオプションを設定する
    if (!cs.sora_video_codec.empty()) {
      json_message["video"]["codec_type"] = cs.sora_video_codec;
    }
    if (cs.sora_video_bitrate != 0) {
      json_message["video"]["bit_rate"] = cs.sora_video_bitrate;
    }
  }

  if (!cs.sora_audio) {
    json_message["audio"] = false;
  } else if (cs.sora_audio && cs.sora_audio_codec.empty() &&
             cs.sora_audio_bitrate == 0) {
    json_message["audio"] = true;
  } else {
    if (!cs.sora_audio_codec.empty()) {
      json_message["audio"]["codec_type"] = cs.sora_audio_codec;
    }
    if (cs.sora_audio_bitrate != 0) {
      json_message["audio"]["bit_rate"] = cs.sora_audio_bitrate;
    }
  }

  ws_->SendText(json_message.dump());
}
void SoraWebsocketClient::doSendPong() {
  json json_message = {{"type", "pong"}};
  ws_->SendText(json_message.dump());
}
void SoraWebsocketClient::doSendPong(
    const rtc::scoped_refptr<const webrtc::RTCStatsReport>& report) {
  std::string stats = report->ToJson();
  json json_message = {{"type", "pong"}, {"stats", stats}};
  std::string str = R"({"type":"pong","stats":)" + stats + "}";
  ws_->SendText(std::move(str));
}

void SoraWebsocketClient::createPeerFromConfig(json jconfig) {
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
  if (conn_settings_.sora_simulcast) {
    rtc_config.set_cpu_adaptation(false);
  }
#endif

  connection_ = manager_->createConnection(rtc_config, this);
}

void SoraWebsocketClient::close() {
  if (ws_->isSSL()) {
    ws_->NativeSecureSocket().async_close(
        boost::beast::websocket::close_code::normal,
        boost::asio::bind_executor(
            ws_->strand(),
            std::bind(&SoraWebsocketClient::onClose, shared_from_this(),
                      std::placeholders::_1)));
  } else {
    ws_->NativeSocket().async_close(
        boost::beast::websocket::close_code::normal,
        boost::asio::bind_executor(
            ws_->strand(),
            std::bind(&SoraWebsocketClient::onClose, shared_from_this(),
                      std::placeholders::_1)));
  }
}

void SoraWebsocketClient::onClose(boost::system::error_code ec) {
  if (ec)
    return MOMO_BOOST_ERROR(ec, "close");
}

void SoraWebsocketClient::onRead(boost::system::error_code ec,
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
    answer_sent_ = false;
    createPeerFromConfig(json_message["config"]);
    const std::string sdp = json_message["sdp"].get<std::string>();

    connection_->setOffer(sdp, [this, json_message]() {
      // simulcast では offer の setRemoteDescription が終わった後に
      // トラックを追加する必要があるため、ここで初期化する
      manager_->initTracks(connection_.get());

      if (conn_settings_.sora_simulcast) {
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
        connection_->setEncodingParameters(std::move(encoding_parameters));
      }

      connection_->createAnswer(
          [this](webrtc::SessionDescriptionInterface* desc) {
            std::string sdp;
            desc->ToString(&sdp);
            json json_message = {{"type", "answer"}, {"sdp", sdp}};
            ws_->SendText(json_message.dump());
          });
    });
  } else if (type == "update") {
    const std::string sdp = json_message["sdp"].get<std::string>();
    connection_->setOffer(sdp, [this]() {
      connection_->createAnswer(
          [this](webrtc::SessionDescriptionInterface* desc) {
            std::string sdp;
            desc->ToString(&sdp);
            json json_message = {{"type", "update"}, {"sdp", sdp}};
            ws_->SendText(json_message.dump());
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
      return;
    }
    watchdog_.Reset();
    bool stats = json_message.value("stats", false);
    if (stats) {
      connection_->getStats(
          [this](
              const rtc::scoped_refptr<const webrtc::RTCStatsReport>& report) {
            doSendPong(report);
          });
    } else {
      doSendPong();
    }
  }
}

// WebRTC からのコールバック
// これらは別スレッドからやってくるので取り扱い注意
void SoraWebsocketClient::onIceConnectionStateChange(
    webrtc::PeerConnectionInterface::IceConnectionState new_state) {
  RTC_LOG(LS_INFO) << __FUNCTION__ << " state:" << new_state;
  boost::asio::post(ws_->strand(),
                    std::bind(&SoraWebsocketClient::doIceConnectionStateChange,
                              shared_from_this(), new_state));
}
void SoraWebsocketClient::onIceCandidate(const std::string sdp_mid,
                                         const int sdp_mlineindex,
                                         const std::string sdp) {
  json json_message = {{"type", "candidate"}, {"candidate", sdp}};
  ws_->SendText(json_message.dump());
}

void SoraWebsocketClient::doIceConnectionStateChange(
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
      reconnectAfter();
      break;
    default:
      break;
  }
  rtc_state_ = new_state;
}
