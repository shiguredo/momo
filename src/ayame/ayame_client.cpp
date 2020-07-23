#include "ayame_client.h"

// boost
#include <boost/beast/websocket/stream.hpp>

// json
#include <nlohmann/json.hpp>

#include "momo_version.h"
#include "ssl_verifier.h"
#include "url_parts.h"
#include "util.h"

using json = nlohmann::json;

bool AyameClient::ParseURL(URLParts& parts) const {
  std::string url = conn_settings_.ayame_signaling_host;

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

boost::asio::ssl::context AyameClient::CreateSSLContext() const {
  boost::asio::ssl::context ctx(boost::asio::ssl::context::tlsv12);
  //ctx.set_default_verify_paths();
  ctx.set_options(boost::asio::ssl::context::default_workarounds |
                  boost::asio::ssl::context::no_sslv2 |
                  boost::asio::ssl::context::no_sslv3 |
                  boost::asio::ssl::context::single_dh_use);
  return ctx;
}

AyameClient::AyameClient(boost::asio::io_context& ioc,
                         RTCManager* manager,
                         ConnectionSettings conn_settings)
    : ioc_(ioc),
      resolver_(ioc),
      manager_(manager),
      retry_count_(0),
      conn_settings_(conn_settings),
      watchdog_(ioc, std::bind(&AyameClient::OnWatchdogExpired, this)) {
  Reset();
}

AyameClient::~AyameClient() {
  destructed_ = true;
  // ここで onIceConnectionStateChange が呼ばれる
  connection_ = nullptr;
}

void AyameClient::Reset() {
  connection_ = nullptr;
  is_send_offer_ = false;
  has_is_exist_user_flag_ = false;
  ice_servers_.clear();

  if (ParseURL(parts_)) {
    auto ssl_ctx = CreateSSLContext();
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

bool AyameClient::Connect() {
  RTC_LOG(LS_INFO) << __FUNCTION__;

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
          std::bind(&AyameClient::OnResolve, shared_from_this(),
                    std::placeholders::_1, std::placeholders::_2)));

  watchdog_.Enable(30);

  return true;
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

void AyameClient::OnResolve(
    boost::system::error_code ec,
    boost::asio::ip::tcp::resolver::results_type results) {
  if (ec) {
    ReconnectAfter();
    return MOMO_BOOST_ERROR(ec, "resolve");
  }

  // DNS ルックアップで得られたエンドポイントに対して接続する
  if (ws_->isSSL()) {
    boost::asio::async_connect(
        ws_->NativeSecureSocket().next_layer().next_layer(), results.begin(),
        results.end(),
        boost::asio::bind_executor(
            ws_->strand(),
            std::bind(&AyameClient::OnSSLConnect, shared_from_this(),
                      std::placeholders::_1)));
  } else {
    boost::asio::async_connect(
        ws_->NativeSocket().next_layer(), results.begin(), results.end(),
        boost::asio::bind_executor(
            ws_->strand(),
            std::bind(&AyameClient::OnConnect, shared_from_this(),
                      std::placeholders::_1)));
  }
}

void AyameClient::OnSSLConnect(boost::system::error_code ec) {
  if (ec) {
    ReconnectAfter();
    return MOMO_BOOST_ERROR(ec, "SSLConnect");
  }

  // SSL のハンドシェイク
  ws_->NativeSecureSocket().next_layer().async_handshake(
      boost::asio::ssl::stream_base::client,
      boost::asio::bind_executor(
          ws_->strand(), std::bind(&AyameClient::OnSSLHandshake,
                                   shared_from_this(), std::placeholders::_1)));
}

void AyameClient::OnSSLHandshake(boost::system::error_code ec) {
  if (ec) {
    ReconnectAfter();
    return MOMO_BOOST_ERROR(ec, "SSLHandshake");
  }

  // Websocket のハンドシェイク
  ws_->NativeSecureSocket().async_handshake(
      parts_.host, parts_.path_query_fragment,
      boost::asio::bind_executor(
          ws_->strand(), std::bind(&AyameClient::OnHandshake,
                                   shared_from_this(), std::placeholders::_1)));
}

void AyameClient::OnConnect(boost::system::error_code ec) {
  if (ec) {
    ReconnectAfter();
    return MOMO_BOOST_ERROR(ec, "Connect");
  }
  // Websocket のハンドシェイク
  ws_->NativeSocket().async_handshake(
      parts_.host, parts_.path_query_fragment,
      boost::asio::bind_executor(
          ws_->strand(), std::bind(&AyameClient::OnHandshake,
                                   shared_from_this(), std::placeholders::_1)));
}

void AyameClient::OnHandshake(boost::system::error_code ec) {
  if (ec) {
    ReconnectAfter();
    return MOMO_BOOST_ERROR(ec, "Handshake");
  }

  ws_->StartToRead(std::bind(&AyameClient::OnRead, this, std::placeholders::_1,
                             std::placeholders::_2, std::placeholders::_3));

  DoRegister();
}

void AyameClient::DoRegister() {
  json json_message = {
      {"type", "register"},
      {"clientId", Util::GenerateRandomChars()},
      {"roomId", conn_settings_.ayame_room_id},
      {"ayameClient", MomoVersion::GetClientName()},
      {"libwebrtc", MomoVersion::GetLibwebrtcName()},
      {"environment", MomoVersion::GetEnvironmentName()},
  };
  if (conn_settings_.ayame_client_id != "") {
    json_message["clientId"] = conn_settings_.ayame_client_id;
  }
  if (conn_settings_.ayame_signaling_key != "") {
    json_message["key"] = conn_settings_.ayame_signaling_key;
  }
  ws_->SendText(json_message.dump());
}

void AyameClient::DoSendPong() {
  json json_message = {{"type", "pong"}};
  ws_->SendText(json_message.dump());
}

void AyameClient::SetIceServersFromConfig(json json_message) {
  // 返却されてきた iceServers を セットする
  if (json_message.contains("iceServers")) {
    auto jservers = json_message["iceServers"];
    if (jservers.is_array()) {
      for (auto jserver : jservers) {
        webrtc::PeerConnectionInterface::IceServer ice_server;
        if (jserver.contains("username")) {
          ice_server.username = jserver["username"].get<std::string>();
        }
        if (jserver.contains("credential")) {
          ice_server.password = jserver["credential"].get<std::string>();
        }
        auto jurls = jserver["urls"];
        for (const std::string url : jurls) {
          ice_server.urls.push_back(url);
          RTC_LOG(LS_INFO) << __FUNCTION__
                           << ": iceserver.url=" << std::string(url);
        }
        ice_servers_.push_back(ice_server);
      }
    }
  }
  if (ice_servers_.empty() && !conn_settings_.no_google_stun) {
    // accept 時に iceServers が返却されてこなかった場合 google の stun server を用いる
    webrtc::PeerConnectionInterface::IceServer ice_server;
    ice_server.uri = "stun:stun.l.google.com:19302";
    ice_servers_.push_back(ice_server);
  }
}

void AyameClient::CreatePeerConnection() {
  webrtc::PeerConnectionInterface::RTCConfiguration rtc_config;

  rtc_config.servers = ice_servers_;
  connection_ = manager_->createConnection(rtc_config, this);
  manager_->initTracks(connection_.get());
}

void AyameClient::Close() {
  // websocket 接続を閉じる
  // 閉じられると OnClose() にコールバックする
  if (ws_->isSSL()) {
    ws_->NativeSecureSocket().async_close(
        boost::beast::websocket::close_code::normal,
        boost::asio::bind_executor(
            ws_->strand(), std::bind(&AyameClient::OnClose, shared_from_this(),
                                     std::placeholders::_1)));
  } else {
    ws_->NativeSocket().async_close(
        boost::beast::websocket::close_code::normal,
        boost::asio::bind_executor(
            ws_->strand(), std::bind(&AyameClient::OnClose, shared_from_this(),
                                     std::placeholders::_1)));
  }
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

  auto json_message = json::parse(text);
  const std::string type = json_message["type"];
  if (type == "accept") {
    SetIceServersFromConfig(json_message);
    CreatePeerConnection();
    // isExistUser フラグが存在するか確認する
    auto is_exist_user = false;
    if (json_message.contains("isExistUser")) {
      has_is_exist_user_flag_ = true;
      is_exist_user = json_message["isExistUser"];
    }

    auto on_create_offer = [this](webrtc::SessionDescriptionInterface* desc) {
      std::string sdp;
      desc->ToString(&sdp);
      json json_message = {{"type", "offer"}, {"sdp", sdp}};
      ws_->SendText(json_message.dump());
    };

    // isExistUser フラグが存在してかつ true な場合 offer SDP を生成して送信する
    if (is_exist_user) {
      RTC_LOG(LS_INFO) << __FUNCTION__ << ": exist_user";
      is_send_offer_ = true;
      connection_->createOffer(on_create_offer);
    } else if (!has_is_exist_user_flag_) {
      // フラグがない場合とりあえず送信
      connection_->createOffer(on_create_offer);
    }
  } else if (type == "offer") {
    // isExistUser フラグがなかった場合二回 peer connection を生成する
    if (!has_is_exist_user_flag_) {
      CreatePeerConnection();
    }
    const std::string sdp = json_message["sdp"];
    connection_->setOffer(sdp, [this]() {
      boost::asio::post(ws_->strand(), [this, self = shared_from_this()]() {
        if (!is_send_offer_ || !has_is_exist_user_flag_) {
          connection_->createAnswer(
              [this](webrtc::SessionDescriptionInterface* desc) {
                std::string sdp;
                desc->ToString(&sdp);
                json json_message = {{"type", "answer"}, {"sdp", sdp}};
                ws_->SendText(json_message.dump());
              });
        }
        is_send_offer_ = false;
      });
    });
  } else if (type == "answer") {
    const std::string sdp = json_message["sdp"];
    connection_->setAnswer(sdp);
  } else if (type == "candidate") {
    int sdp_mlineindex = 0;
    std::string sdp_mid, candidate;
    json ice = json_message["ice"];
    sdp_mid = ice["sdpMid"].get<std::string>();
    sdp_mlineindex = ice["sdpMLineIndex"].get<int>();
    candidate = ice["candidate"].get<std::string>();
    connection_->addIceCandidate(sdp_mid, sdp_mlineindex, candidate);
  } else if (type == "ping") {
    watchdog_.Reset();
    DoSendPong();
  } else if (type == "bye") {
    RTC_LOG(LS_INFO) << __FUNCTION__ << ": bye";
    connection_ = nullptr;
    Close();
  }
}

// WebRTC からのコールバック
// これらは別スレッドからやってくるので取り扱い注意
void AyameClient::onIceConnectionStateChange(
    webrtc::PeerConnectionInterface::IceConnectionState new_state) {
  RTC_LOG(LS_INFO) << __FUNCTION__ << " state:" << new_state;
  // デストラクタだと shared_from_this が機能しないので無視する
  if (destructed_) {
    return;
  }
  boost::asio::post(ws_->strand(),
                    std::bind(&AyameClient::DoIceConnectionStateChange,
                              shared_from_this(), new_state));
}
void AyameClient::onIceCandidate(const std::string sdp_mid,
                                 const int sdp_mlineindex,
                                 const std::string sdp) {
  // ayame では candidate sdp の交換で `ice` プロパティを用いる。 `candidate` ではないので注意
  json json_message = {
      {"type", "candidate"},
  };
  // ice プロパティの中に object で candidate 情報をセットして送信する
  json_message["ice"] = {{"candidate", sdp},
                         {"sdpMLineIndex", sdp_mlineindex},
                         {"sdpMid", sdp_mid}};
  ws_->SendText(json_message.dump());
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