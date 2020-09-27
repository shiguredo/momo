#include "p2p_websocket_session.h"

// Boost
#include <boost/asio/bind_executor.hpp>
#include <boost/beast/websocket/error.hpp>
#include <boost/beast/websocket/stream.hpp>

// nlohmann/json
#include <nlohmann/json.hpp>

#include "util.h"

using json = nlohmann::json;

P2PWebsocketSession::P2PWebsocketSession(boost::asio::io_context& ioc,
                                         boost::asio::ip::tcp::socket socket,
                                         RTCManager* rtc_manager,
                                         P2PWebsocketSessionConfig config)
    : ws_(new Websocket(std::move(socket))),
      rtc_manager_(rtc_manager),
      config_(std::move(config)),
      watchdog_(ioc, std::bind(&P2PWebsocketSession::OnWatchdogExpired, this)) {
  RTC_LOG(LS_INFO) << __FUNCTION__;
}

P2PWebsocketSession::~P2PWebsocketSession() {
  RTC_LOG(LS_INFO) << __FUNCTION__;
}

void P2PWebsocketSession::Run(
    boost::beast::http::request<boost::beast::http::string_body> req) {
  RTC_LOG(LS_INFO) << __FUNCTION__;
  DoAccept(std::move(req));
}

void P2PWebsocketSession::OnWatchdogExpired() {
  json ping_message = {
      {"type", "ping"},
  };
  ws_->WriteText(std::move(ping_message.dump()));
  watchdog_.Reset();
}

void P2PWebsocketSession::DoAccept(
    boost::beast::http::request<boost::beast::http::string_body> req) {
  ws_->Accept(std::move(req),
              std::bind(&P2PWebsocketSession::OnAccept, shared_from_this(),
                        std::placeholders::_1));
}

void P2PWebsocketSession::OnAccept(boost::system::error_code ec) {
  RTC_LOG(LS_INFO) << __FUNCTION__ << ": " << ec;

  if (ec)
    return MOMO_BOOST_ERROR(ec, "Accept");

  DoRead();
}

void P2PWebsocketSession::DoRead() {
  ws_->Read(std::bind(&P2PWebsocketSession::OnRead, shared_from_this(),
                      std::placeholders::_1, std::placeholders::_2,
                      std::placeholders::_3));
}

void P2PWebsocketSession::OnRead(boost::system::error_code ec,
                                 std::size_t bytes_transferred,
                                 std::string recv_string) {
  RTC_LOG(LS_INFO) << __FUNCTION__ << ": " << ec;

  boost::ignore_unused(bytes_transferred);

  if (ec == boost::beast::websocket::error::closed)
    return;

  if (ec)
    return MOMO_BOOST_ERROR(ec, "Read");

  // これ以降はエラーが起きて処理を中断したとしても DoRead() する
  struct Guard {
    std::function<void()> f;
    ~Guard() { f(); }
  } guard = {[this]() { DoRead(); }};

  json recv_message;

  RTC_LOG(LS_INFO) << __FUNCTION__ << ": recv_string=" << recv_string;

  try {
    recv_message = json::parse(recv_string);
  } catch (json::parse_error& e) {
    return;
  }

  std::string type;
  try {
    type = recv_message["type"].get<std::string>();
  } catch (json::type_error& e) {
    return;
  }

  if (type == "offer") {
    std::string sdp;
    try {
      sdp = recv_message["sdp"].get<std::string>();
    } catch (json::type_error& e) {
      return;
    }

    connection_ = CreateRTCConnection();
    connection_->SetOffer(sdp, [this]() {
      connection_->CreateAnswer(
          [this](webrtc::SessionDescriptionInterface* desc) {
            std::string sdp;
            desc->ToString(&sdp);
            json json_desc = {{"type", "answer"}, {"sdp", sdp}};
            std::string str_desc = json_desc.dump();
            ws_->WriteText(std::move(str_desc));
          });
    });
  } else if (type == "answer") {
    if (!connection_) {
      return;
    }
    std::string sdp;
    try {
      sdp = recv_message["sdp"].get<std::string>();
    } catch (json::type_error& e) {
      return;
    }
    connection_->SetAnswer(sdp);
  } else if (type == "candidate") {
    if (!connection_) {
      return;
    }
    int sdp_mlineindex = 0;
    std::string sdp_mid, candidate;
    try {
      json ice = recv_message["ice"];
      sdp_mid = ice["sdpMid"].get<std::string>();
      sdp_mlineindex = ice["sdpMLineIndex"].get<int>();
      candidate = ice["candidate"].get<std::string>();
    } catch (json::type_error& e) {
      return;
    }
    connection_->AddIceCandidate(sdp_mid, sdp_mlineindex, candidate);
  } else if (type == "close" || type == "bye") {
    connection_ = nullptr;
  } else if (type == "register") {
    json accept_message = {
        {"type", "accept"},
        {"isExistUser", true},
    };
    ws_->WriteText(std::move(accept_message.dump()));
    watchdog_.Enable(30);
  } else {
    return;
  }
}

std::shared_ptr<RTCConnection> P2PWebsocketSession::CreateRTCConnection() {
  webrtc::PeerConnectionInterface::RTCConfiguration rtc_config;
  webrtc::PeerConnectionInterface::IceServers servers;
  if (!config_.no_google_stun) {
    webrtc::PeerConnectionInterface::IceServer ice_server;
    ice_server.uri = "stun:stun.l.google.com:19302";
    servers.push_back(ice_server);
  }
  rtc_config.servers = servers;
  auto connection = rtc_manager_->CreateConnection(rtc_config, this);
  rtc_manager_->InitTracks(connection.get());

  return connection;
}

void P2PWebsocketSession::OnIceConnectionStateChange(
    webrtc::PeerConnectionInterface::IceConnectionState new_state) {
  RTC_LOG(LS_INFO) << __FUNCTION__ << " rtc_state "
                   << Util::IceConnectionStateToString(rtc_state_) << " -> "
                   << Util::IceConnectionStateToString(new_state);

  rtc_state_ = new_state;
}

void P2PWebsocketSession::OnIceCandidate(const std::string sdp_mid,
                                         const int sdp_mlineindex,
                                         const std::string sdp) {
  RTC_LOG(LS_INFO) << __FUNCTION__;

  json json_cand = {{"type", "candidate"}};
  json_cand["ice"] = {{"candidate", sdp},
                      {"sdpMLineIndex", sdp_mlineindex},
                      {"sdpMid", sdp_mid}};
  std::string str_cand = json_cand.dump();
  ws_->WriteText(str_cand);
}
