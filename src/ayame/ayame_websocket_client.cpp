#include "ayame_websocket_client.h"

#include <boost/beast/websocket/stream.hpp>

#include <nlohmann/json.hpp>
#include "url_parts.h"
#include "util.h"

using json = nlohmann::json;

bool AyameWebsocketClient::parseURL(URLParts& parts) const {
    std::string url = conn_settings_.ayame_signaling_host;

    if (!URLParts::parse(url, parts)) {
        throw std::exception();
    }

    std::string default_port;
    if (parts.scheme =d= "wss") {
        return true;
    } else if (parts.scheme == "ws") {
        return false;
    } else {
        throw std::exception();
    }
}

boost::asio::ssl::context AyameWebsocketClient::createSSLContext() const {
    boost::asio::ssl::context ctx(boost::asio::ssl::context::tlsv12);
    ctx.set_default_verify_paths();
    ctx.set_options(boost::asio::ssl::context::default_workarounds |
                    boost::asio::ssl::context::no_sslv2 |
                    boost::asio::ssl::context::no_sslv3 |
                    boost::asio::ssl::context::single_dh_use);
    return ctx;
}

webrtc::PeerConnectionInterface::IceConnectionState AyameWebsocketClient::getRTCConnectionState() const { return rtc_state_; }

std::shared_ptr<RTCConnection> AyameWebsocketClient::getRTCConnection() const {
  if (rtc_state_ == webrtc::PeerConnectionInterface::IceConnectionState::kIceConnectionConnected) {
    return connection_;
  } else {
    return nullptr;
  }
}

AyameWebsocketClient::AyameWebsocketClient(boost::asio::io_context& ioc, RTCManager* manager, ConnectionSettings conn_settings)
    : ioc_(ioc)
    , resolver_(ioc)
    , manager_(manager)
    , retry_count_(0)
    , conn_settings_(conn_settings)
    , watchdog_(ioc, std::bind(&AyameWebsocketClient::onWatchdogExpired, this))
{
    reset();
}

void AyameWebsocketClient::reset() {
    connection_ = nullptr;
    connected_ = false;

    if (parseURL(parts_)) {
        auto ssl_ctx = createSSLContext();
        boost::beast::websocket::stream<boost::asio::ssl::stream<boost::asio::ip::tcp::socket>> wss(ioc_, ssl_ctx);
        ws_.reset(new Websocket(ioc_, std::move(ssl_ctx)));
        // SNI の設定を行う
        if (!SSL_set_tlsext_host_name(ws_->nativeSecureSocket().next_layer().native_handle(), parts_.host.c_str()))
        {
            boost::system::error_code ec{static_cast<int>(::ERR_get_error()), boost::asio::error::get_ssl_category()};
            MOMO_BOOST_ERROR(ec, "SSL_set_tlsext_host_name");
        }
    } else {
        boost::beast::websocket::stream<boost::asio::ip::tcp::socket> ws(ioc_);
        ws_.reset(new Websocket(ioc_));
    }
}

void AyameWebsocketClient::release() {
    connection_ = nullptr;
}

bool AyameWebsocketClient::connect()
{
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
        parts_.host,
        port,
        boost::asio::bind_executor(
            ws_->strand(),
            std::bind(
                &AyameWebsocketClient::onResolve,
                shared_from_this(),
                std::placeholders::_1,
                std::placeholders::_2)));

    watchdog_.enable(30);

    return true;
}

void AyameWebsocketClient::reconnectAfter()
{
  int interval = 5 * (2 * retry_count_ + 1);
  RTC_LOG(LS_INFO) << __FUNCTION__ << " reconnect after " << interval <<" sec";

  watchdog_.enable(interval);
  retry_count_++;
}

void AyameWebsocketClient::onWatchdogExpired() {
    RTC_LOG(LS_WARNING) << __FUNCTION__;

    RTC_LOG(LS_INFO) << __FUNCTION__ << " reconnecting...:";
    reset();
    connect();
}

void AyameWebsocketClient::onResolve(boost::system::error_code ec, boost::asio::ip::tcp::resolver::results_type results) {
    if (ec) {
        reconnectAfter();
        return MOMO_BOOST_ERROR(ec, "resolve");
    }

    // DNS ルックアップで得られたエンドポイントに対して接続する
    if (ws_->isSSL()) {
        boost::asio::async_connect(
            ws_->nativeSecureSocket().next_layer().next_layer(),
            results.begin(),
            results.end(),
            boost::asio::bind_executor(
                ws_->strand(),
                std::bind(
                    &AyameWebsocketClient::onSSLConnect,
                    shared_from_this(),
                    std::placeholders::_1)));
    } else {
        boost::asio::async_connect(
            ws_->nativeSocket().next_layer(),
            results.begin(),
            results.end(),
            boost::asio::bind_executor(
                ws_->strand(),
                std::bind(
                    &AyameWebsocketClient::onConnect,
                    shared_from_this(),
                    std::placeholders::_1)));
    }
}

void AyameWebsocketClient::onSSLConnect(boost::system::error_code ec) {
    if (ec) {
        reconnectAfter();
        return MOMO_BOOST_ERROR(ec, "SSLConnect");
    }

    // SSL のハンドシェイク
    ws_->nativeSecureSocket().next_layer().async_handshake(
        boost::asio::ssl::stream_base::client,
        boost::asio::bind_executor(
            ws_->strand(),
            std::bind(
                &AyameWebsocketClient::onSSLHandshake,
                shared_from_this(),
                std::placeholders::_1)));
}

void AyameWebsocketClient::onSSLHandshake(boost::system::error_code ec) {
    if (ec) {
        reconnectAfter();
        return MOMO_BOOST_ERROR(ec, "SSLHandshake");
    }

    // Websocket のハンドシェイク
    ws_->nativeSecureSocket().async_handshake(parts_.host, parts_.path_query_fragment,
        boost::asio::bind_executor(
            ws_->strand(),
            std::bind(
                &AyameWebsocketClient::onHandshake,
                shared_from_this(),
                std::placeholders::_1)));
}

void AyameWebsocketClient::onConnect(boost::system::error_code ec) {
    if (ec) {
        reconnectAfter();
        return MOMO_BOOST_ERROR(ec, "connect");
    }

    // Websocket のハンドシェイク
    ws_->nativeSocket().async_handshake(parts_.host, parts_.path_query_fragment,
        boost::asio::bind_executor(
            ws_->strand(),
            std::bind(
                &AyameWebsocketClient::onHandshake,
                shared_from_this(),
                std::placeholders::_1)));
}

void AyameWebsocketClient::onHandshake(boost::system::error_code ec)
{
    if (ec) {
        reconnectAfter();
        return MOMO_BOOST_ERROR(ec, "Handshake");
    }

    connected_ = true;

    ws_->startToRead(std::bind(
        &AyameWebsocketClient::onRead,
        this,
        std::placeholders::_1,
        std::placeholders::_2,
        std::placeholders::_3));

    doRegister();
}

void AyameWebsocketClient::doRegister()
{
    json json_message = {
        {"type", "register"},
        {"client_id", conn_settings_.ayame_client_id},
        {"room_id", conn_settings_.ayame_room_id}
    };
    ws_->sendText(json_message.dump());
}
void AyameWebsocketClient::doSendPong() {
    json json_message = {
      {"type", "pong"}
    };
    ws_->sendText(json_message.dump());
}

// TODO(kdxu): iceServers などの設定を外出しする
void AyameWebsocketClient::createPeerConnection()
{
  webrtc::PeerConnectionInterface::RTCConfiguration rtc_config;
  webrtc::PeerConnectionInterface::IceServers ice_servers;

  auto jservers = {
    "urls": ["stun:stun.l.google.com:19302"]
  };
  for (auto jserver : jservers)
  {
    auto jurls = jserver["urls"];
    for (const std::string url : jurls)
    {
      webrtc::PeerConnectionInterface::IceServer ice_server;
      ice_server.uri = url;
      ice_servers.push_back(ice_server);
    }
  }

  rtc_config.servers = ice_servers;

  connection_ = manager_->createConnection(rtc_config, this);
}

void AyameWebsocketClient::close()
{
    if (ws_->isSSL()) {
        ws_->nativeSecureSocket().async_close(boost::beast::websocket::close_code::normal,
            boost::asio::bind_executor(
                ws_->strand(),
                std::bind(
                    &AyameWebsocketClient::onClose,
                    shared_from_this(),
                    std::placeholders::_1)));
    } else {
        ws_->nativeSocket().async_close(boost::beast::websocket::close_code::normal,
            boost::asio::bind_executor(
                ws_->strand(),
                std::bind(
                    &AyameWebsocketClient::onClose,
                    shared_from_this(),
                    std::placeholders::_1)));
    }
}

void AyameWebsocketClient::onClose(boost::system::error_code ec)
{
    if (ec)
        return MOMO_BOOST_ERROR(ec, "close");
}

void AyameWebsocketClient::onRead(boost::system::error_code ec, std::size_t bytes_transferred, std::string text)
{
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
    if (type == "accept")
    {
      if (!connection_) {
        createPeerConnection();
      }
    }
    if (type == "offer") {
      if (!connection_) {
        // accept よりあとに来るので、この時点で peer connection は生成されているべき
        return;
      }
      const std::string sdp = json_message["sdp"];
      connection_->setOffer(sdp);
    else if (type == "answer") {
        if (!connection_) {
            return;
        }
        std::string sdp;
        try
        {
            sdp = json_message["sdp"];
        }
        catch (json::type_error &e)
        {
            return;
        }
        connection_->setAnswer(sdp);
    }
    else if (type == "candidate") {
        if (!connection_) {
            return;
        }
        if (rtc_state_ == webrtc::PeerConnectionInterface::IceConnectionState::kIceConnectionConnected) {
          int sdp_mlineindex = 0;
          std::string sdp_mid, candidate;
          json ice = json_message["ice"];
          sdp_mid = ice["sdpMid"];
          sdp_mlineindex = ice["sdpMLineIndex"];
          candidate = ice["candidate"];
          connection_->addIceCandidate(sdp_mid, sdp_mlineindex, candidate);
        }
    } else if (type == "ping") {
      if (!connection_) {
        return;
      }
      if (rtc_state_ != webrtc::PeerConnectionInterface::IceConnectionState::kIceConnectionConnected)
      {
        return;
      }
      watchdog_.reset();
      doSendPong();
    }
}

// WebRTC からのコールバック
// これらは別スレッドからやってくるので取り扱い注意
void AyameWebsocketClient::onIceConnectionStateChange(webrtc::PeerConnectionInterface::IceConnectionState new_state) {
    RTC_LOG(LS_INFO) << __FUNCTION__ << " state:" << new_state;
    boost::asio::post(
        ws_->strand(),
        std::bind(
            &AyameWebsocketClient::doIceConnectionStateChange,
            shared_from_this(),
            new_state));
}
void AyameWebsocketClient::onIceCandidate(const std::string sdp_mid, const int sdp_mlineindex, const std::string sdp) {
    json json_message = {
      {"type", "candidate"},
      {"candidate", sdp}
    };
    ws_->sendText(json_message.dump());
}
void AyameWebsocketClient::onCreateDescription(webrtc::SdpType type, const std::string sdp) {
    json json_message = {
      {"type", "answer"},
      {"sdp", sdp}
    };
    ws_->sendText(json_message.dump());
}
void AyameWebsocketClient::onSetDescription(webrtc::SdpType type) {
  RTC_LOG(LS_INFO) << __FUNCTION__ << " SdpType: " << webrtc::SdpTypeToString(type);
  if (type == webrtc::SdpType::kOffer) {
    connection_->createAnswer();
  }
}

void AyameWebsocketClient::doIceConnectionStateChange(webrtc::PeerConnectionInterface::IceConnectionState new_state) {
    RTC_LOG(LS_INFO) << __FUNCTION__ << ": newState=" << Util::iceConnectionStateToString(new_state);

    switch (new_state) {
      case webrtc::PeerConnectionInterface::IceConnectionState::kIceConnectionConnected:
        retry_count_ = 0;
        watchdog_.enable(60);
        break;
      case webrtc::PeerConnectionInterface::IceConnectionState::kIceConnectionFailed:
        reconnectAfter();
        break;
      default:
        break;
    }
    rtc_state_ = new_state;
}
