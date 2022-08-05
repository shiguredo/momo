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
#include "zlib_helper.h"

bool SoraClient::ParseURL(const std::string& url,
                          URLParts& parts,
                          bool& ssl) const {
  if (!URLParts::Parse(url, parts)) {
    return false;
  }

  if (parts.scheme == "wss") {
    ssl = true;
    return true;
  } else if (parts.scheme == "ws") {
    ssl = false;
    return true;
  } else {
    return false;
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
  // 一応閉じる努力はする
  if (using_datachannel_ && dc_) {
    webrtc::DataBuffer disconnect =
        ConvertToDataBuffer("signaling", R"({"type":"disconnect"})");
    dc_->Close(
        disconnect, [dc = dc_]() {}, config_.disconnect_wait_timeout);
    dc_ = nullptr;
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
  }
  // ここで OnIceConnectionStateChange が呼ばれる
  connection_ = nullptr;
}

void SoraClient::Close(std::function<void()> on_close) {
  auto dc = std::move(dc_);
  dc_ = nullptr;
  auto ws = std::move(ws_);
  ws_ = nullptr;
  auto connection = std::move(connection_);
  connection_ = nullptr;

  if (using_datachannel_ && ws) {
    webrtc::DataBuffer disconnect =
        ConvertToDataBuffer("signaling", R"({"type":"disconnect"})");
    dc->Close(
        disconnect,
        [self = shared_from_this(), dc, connection, ws = std::move(ws),
         on_close]() {
          ws->Close(
              [self, ws, on_close](boost::system::error_code) { on_close(); });
        },
        config_.disconnect_wait_timeout);
  } else if (using_datachannel_ && !ws) {
    webrtc::DataBuffer disconnect =
        ConvertToDataBuffer("signaling", R"({"type":"disconnect"})");
    dc->Close(
        disconnect, [dc, connection, on_close]() { on_close(); },
        config_.disconnect_wait_timeout);
  } else if (!using_datachannel_ && ws) {
    boost::json::value disconnect = {{"type", "disconnect"}};
    ws->WriteText(boost::json::serialize(disconnect),
                  [self = shared_from_this(), ws](boost::system::error_code,
                                                  std::size_t) {});
    ws->Close([self = shared_from_this(), ws,
               on_close](boost::system::error_code) { on_close(); });
  } else {
    on_close();
  }
}

void SoraClient::Reset() {
  watchdog_.Disable();
  connection_ = nullptr;

  connecting_wss_.clear();
  ws_.reset();
  dc_.reset(new SoraDataChannelOnAsio(ioc_, this));
}

void SoraClient::Connect() {
  RTC_LOG(LS_INFO) << __FUNCTION__;

  watchdog_.Enable(30);

  ws_.reset();
  connecting_wss_.clear();

  for (const auto& url : config_.signaling_urls) {
    URLParts parts;
    bool ssl;
    if (!ParseURL(url, parts, ssl)) {
      RTC_LOG(LS_ERROR) << "Invalid URL: url=" << url;
      continue;
    }

    std::shared_ptr<Websocket> ws;
    if (ssl) {
      if (config_.proxy_url.empty()) {
        ws.reset(new Websocket(Websocket::ssl_tag(), ioc_, config_.insecure,
                               config_.client_cert, config_.client_key));
      } else {
        ws.reset(new Websocket(Websocket::https_proxy_tag(), ioc_,
                               config_.insecure, config_.client_cert,
                               config_.client_key, config_.proxy_url,
                               config_.proxy_username, config_.proxy_password));
      }
    } else {
      ws.reset(new Websocket(ioc_));
    }
    ws->Connect(url, std::bind(&SoraClient::OnConnect, shared_from_this(),
                               std::placeholders::_1, url, ws));
    connecting_wss_.push_back(ws);
  }
}

void SoraClient::ReconnectAfter() {
  int interval = 5 * (2 * retry_count_ + 1);
  RTC_LOG(LS_INFO) << __FUNCTION__ << " reconnect after " << interval << " sec";

  watchdog_.Enable(interval);
  retry_count_++;
}

void SoraClient::OnWatchdogExpired() {
  RTC_LOG(LS_INFO) << __FUNCTION__ << " closing...";
  Close([this]() {
    RTC_LOG(LS_INFO) << __FUNCTION__ << " closed and reconnecting...";
    Reset();
    Connect();
  });
}

void SoraClient::OnConnect(boost::system::error_code ec,
                           std::string url,
                           std::shared_ptr<Websocket> ws) {
  connecting_wss_.erase(
      std::remove_if(connecting_wss_.begin(), connecting_wss_.end(),
                     [ws](std::shared_ptr<Websocket> p) { return p == ws; }),
      connecting_wss_.end());
  if (ec) {
    // すべての接続がうまくいかなかったら再接続フローに入る
    if (!ws_ && connecting_wss_.empty()) {
      ReconnectAfter();
    }
    return MOMO_BOOST_ERROR(ec, "Handshake");
  }
  if (ws_) {
    // 既に他の接続が先に完了していたので、切断する
    ws->Close([self = shared_from_this(), ws](boost::system::error_code) {});
    return;
  }

  ws_ = ws;
  connected_signaling_url_ = url;
  RTC_LOG(LS_INFO) << "connected: url=" << url;

  DoRead();
  DoSendConnect(false);
}

void SoraClient::Redirect(std::string url) {
  ws_->Read([self = shared_from_this(), url](boost::system::error_code ec,
                                             std::size_t bytes_transferred,
                                             std::string text) {
    auto on_close = [self, url](boost::system::error_code ec) {
      // close 処理に成功してても失敗してても処理は続ける
      URLParts parts;
      bool ssl;
      if (!self->ParseURL(url, parts, ssl)) {
        RTC_LOG(LS_ERROR) << "Invalid URL: url=" << url;
        return;
      }

      std::shared_ptr<Websocket> ws;
      if (ssl) {
        ws.reset(new Websocket(
            Websocket::ssl_tag(), self->ioc_, self->config_.insecure,
            self->config_.client_cert, self->config_.client_key));
      } else {
        ws.reset(new Websocket(self->ioc_));
      }
      ws->Connect(url, std::bind(&SoraClient::OnRedirect, self,
                                 std::placeholders::_1, url, ws));
    };

    // type: redirect の後、サーバは切断してるはずなので、正常に処理が終わるのはおかしい
    if (!ec) {
      RTC_LOG(LS_WARNING) << "Unexpected success to read";
      // 強制的に閉じる
      self->ws_->Close(on_close);
      return;
    }
    on_close(
        boost::system::errc::make_error_code(boost::system::errc::success));
  });
}
void SoraClient::OnRedirect(boost::system::error_code ec,
                            std::string url,
                            std::shared_ptr<Websocket> ws) {
  if (ec) {
    ReconnectAfter();
    return MOMO_BOOST_ERROR(ec, "Handshake");
  }

  ws_ = ws;
  connected_signaling_url_ = url;
  RTC_LOG(LS_INFO) << "redirected: url=" << url;

  DoRead();
  DoSendConnect(true);
}

void SoraClient::DoRead() {
  ws_->Read([self = shared_from_this(), ws = ws_](boost::system::error_code ec,
                                                  std::size_t bytes_transferred,
                                                  std::string text) {
    self->OnRead(ec, bytes_transferred, std::move(text));
  });
}

void SoraClient::DoSendConnect(bool redirect) {
  boost::json::object json_message = {
      {"type", "connect"},
      {"role", config_.role},
      {"channel_id", config_.channel_id},
      {"sora_client", MomoVersion::GetClientName()},
      {"libwebrtc", MomoVersion::GetLibwebrtcName()},
      {"environment", MomoVersion::GetEnvironmentName()},
  };

  if (redirect) {
    json_message["redirect"] = true;
  }

  json_message["multistream"] = config_.multistream;

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

  if (config_.data_channel_signaling) {
    json_message["data_channel_signaling"] = *config_.data_channel_signaling;
  }
  if (config_.ignore_disconnect_websocket) {
    json_message["ignore_disconnect_websocket"] =
        *config_.ignore_disconnect_websocket;
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
  if (dc_ && using_datachannel_ && dc_->IsOpen("stats")) {
    // DataChannel が使える場合は type: stats で DataChannel に送る
    std::string str = R"({"type":"stats","reports":)" + stats + "}";
    SendDataChannel("stats", str);
  } else if (ws_) {
    std::string str = R"({"type":"pong","stats":)" + stats + "}";
    ws_->WriteText(std::move(str));
  }
}
void SoraClient::DoSendUpdate(const std::string& sdp, std::string type) {
  boost::json::value json_message = {{"type", type}, {"sdp", sdp}};
  if (dc_ && using_datachannel_ && dc_->IsOpen("signaling")) {
    // DataChannel が使える場合は DataChannel に送る
    SendDataChannel("signaling", boost::json::serialize(json_message));
  } else if (ws_) {
    ws_->WriteText(boost::json::serialize(json_message));
  }
}

std::shared_ptr<RTCConnection> SoraClient::CreateRTCConnection(
    boost::json::value jconfig) {
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

  manager_->AddDataManager(dc_);
  return manager_->CreateConnection(rtc_config, this);
}

void SoraClient::OnRead(boost::system::error_code ec,
                        std::size_t bytes_transferred,
                        std::string text) {
  RTC_LOG(LS_INFO) << __FUNCTION__ << ": " << ec;

  boost::ignore_unused(bytes_transferred);

  // 書き込みのために読み込み処理がキャンセルされた時にこのエラーになるので、これはエラーとして扱わない
  if (ec == boost::asio::error::operation_aborted)
    return;

  if (ec) {
    // とりあえず WS や DC を閉じておいて、後で再接続が起きるようにする
    ReconnectAfter();
    Close([]() {});
    return MOMO_BOOST_ERROR(ec, "Read");
  }

  RTC_LOG(LS_INFO) << __FUNCTION__ << ": text=" << text;

  auto json_message = boost::json::parse(text);
  const std::string type = json_message.at("type").as_string().c_str();
  if (type == "redirect") {
    const std::string location =
        json_message.at("location").as_string().c_str();
    Redirect(location);
    // Redirect の中で次の Read をしているのでここで return する
    return;
  } else if (type == "offer") {
    // Data Channel の圧縮されたデータが送られてくるラベルを覚えておく
    {
      auto it = json_message.as_object().find("data_channels");
      if (it != json_message.as_object().end()) {
        const auto& ar = it->value().as_array();
        for (const auto& v : ar) {
          if (v.at("compress").as_bool()) {
            compressed_labels_.insert(v.at("label").as_string().c_str());
          }
        }
      }
    }

    connection_ = CreateRTCConnection(json_message.at("config"));
    const std::string sdp = json_message.at("sdp").as_string().c_str();

    connection_->SetOffer(sdp, [self = shared_from_this(), json_message]() {
      boost::asio::post(self->ioc_, [self, json_message]() {
        if (!self->connection_) {
          return;
        }

        // simulcast では offer の setRemoteDescription が終わった後に
        // トラックを追加する必要があるため、ここで初期化する
        self->manager_->InitTracks(self->connection_.get());

        if (self->config_.simulcast &&
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

          std::string mid;
          {
            // TODO(melpon): しばらく mid が無い可能性も考慮するが、そのうち必須にする
            auto it = json_message.as_object().find("mid");
            if (it != json_message.as_object().end()) {
              const auto& midobj = it->value().as_object();
              // video: false の場合は video フィールドが mid が無いのでチェックする
              it = midobj.find("video");
              if (it != midobj.end()) {
                mid = it->value().as_string().c_str();
              }
            }
          }
          RTC_LOG(LS_INFO) << "mid: " << mid;
          self->connection_->SetEncodingParameters(
              mid, std::move(encoding_parameters));
        }

        self->connection_->CreateAnswer(
            [self](webrtc::SessionDescriptionInterface* desc) {
              std::string sdp;
              desc->ToString(&sdp);
              self->manager_->SetParameters();
              boost::asio::post(self->ioc_, [self, sdp]() {
                if (!self->connection_) {
                  return;
                }

                boost::json::value json_message = {{"type", "answer"},
                                                   {"sdp", sdp}};
                self->ws_->WriteText(boost::json::serialize(json_message));
              });
            });
      });
    });
  } else if (type == "update" || type == "re-offer") {
    if (connection_ == nullptr) {
      return;
    }
    std::string answer_type = type == "update" ? "update" : "re-answer";
    const std::string sdp = json_message.at("sdp").as_string().c_str();
    connection_->SetOffer(sdp, [self = shared_from_this(), answer_type]() {
      boost::asio::post(self->ioc_, [self, answer_type]() {
        if (!self->connection_) {
          return;
        }

        // エンコーディングパラメータの情報がクリアされるので設定し直す
        if (self->config_.simulcast) {
          self->connection_->ResetEncodingParameters();
        }

        self->connection_->CreateAnswer(
            [self, answer_type](webrtc::SessionDescriptionInterface* desc) {
              std::string sdp;
              desc->ToString(&sdp);
              self->manager_->SetParameters();
              boost::asio::post(self->ioc_, [self, sdp, answer_type]() {
                if (!self->connection_) {
                  return;
                }

                self->DoSendUpdate(sdp, answer_type);
              });
            });
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
          [self = shared_from_this()](
              const rtc::scoped_refptr<const webrtc::RTCStatsReport>& report) {
            self->DoSendPong(report);
          });
    } else {
      DoSendPong();
    }
  } else if (type == "switched") {
    // Data Channel による通信の開始
    using_datachannel_ = true;

    // ignore_disconnect_websocket == true の場合は WS を切断する
    auto it = json_message.as_object().find("ignore_disconnect_websocket");
    if (it != json_message.as_object().end() && it->value().as_bool() && ws_) {
      RTC_LOG(LS_INFO) << "Close WebSocket for DataChannel";
      auto ws = ws_;
      ws_ = nullptr;
      ws->Close([self = shared_from_this(), ws](boost::system::error_code) {});

      watchdog_.Enable(config_.data_channel_signaling_timeout);
      return;
    }
  }
  DoRead();
}

webrtc::DataBuffer SoraClient::ConvertToDataBuffer(const std::string& label,
                                                   const std::string& input) {
  bool compressed = compressed_labels_.find(label) != compressed_labels_.end();
  RTC_LOG(LS_INFO) << "Convert to DataBuffer: label=" << label
                   << " compressed=" << compressed << " input=" << input;
  const std::string& str = compressed ? ZlibHelper::Compress(input) : input;
  return webrtc::DataBuffer(rtc::CopyOnWriteBuffer(str), compressed);
}

void SoraClient::SendDataChannel(const std::string& label,
                                 const std::string& input) {
  webrtc::DataBuffer data = ConvertToDataBuffer(label, input);
  dc_->Send(label, data);
}

void SoraClient::OnStateChange(
    rtc::scoped_refptr<webrtc::DataChannelInterface> data_channel) {}

void SoraClient::OnMessage(
    rtc::scoped_refptr<webrtc::DataChannelInterface> data_channel,
    const webrtc::DataBuffer& buffer) {
  if (!using_datachannel_ || !dc_) {
    return;
  }

  std::string label = data_channel->label();
  bool compressed = compressed_labels_.find(label) != compressed_labels_.end();
  std::string data;
  if (compressed) {
    data = ZlibHelper::Uncompress(buffer.data.cdata(), buffer.size());
  } else {
    data.assign((const char*)buffer.data.cdata(),
                (const char*)buffer.data.cdata() + buffer.size());
  }

  RTC_LOG(LS_INFO) << "label=" << label << " data=" << data;

  // ハンドリングする必要のあるラベル以外は何もしない
  if (label != "signaling" && label != "stats") {
    return;
  }

  boost::json::error_code ec;
  auto json = boost::json::parse(data, ec);
  if (ec) {
    RTC_LOG(LS_ERROR) << "JSON Parse Error ec=" << ec.message();
    return;
  }

  watchdog_.Reset();

  if (label == "signaling") {
    const std::string type = json.at("type").as_string().c_str();
    if (type == "re-offer") {
      const std::string sdp = json.at("sdp").as_string().c_str();
      connection_->SetOffer(sdp, [self = shared_from_this()]() {
        boost::asio::post(self->ioc_, [self]() {
          if (!self->connection_) {
            return;
          }

          // エンコーディングパラメータの情報がクリアされるので設定し直す
          if (self->config_.simulcast) {
            self->connection_->ResetEncodingParameters();
          }

          self->connection_->CreateAnswer(
              [self](webrtc::SessionDescriptionInterface* desc) {
                std::string sdp;
                desc->ToString(&sdp);
                boost::asio::post(self->ioc_, [self, sdp]() {
                  if (!self->connection_) {
                    return;
                  }
                  self->DoSendUpdate(sdp, "re-answer");
                });
              });
        });
      });
    }
  }

  if (label == "stats") {
    connection_->GetStats(
        [self = shared_from_this()](
            const rtc::scoped_refptr<const webrtc::RTCStatsReport>& report) {
          self->DoSendPong(report);
        });
  }
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
