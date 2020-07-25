#ifndef P2P_WEBSOCKET_SESSION_H_
#define P2P_WEBSOCKET_SESSION_H_

#include <cstdlib>
#include <memory>
#include <string>

// Boost
#include <boost/asio/ip/tcp.hpp>
#include <boost/beast/core/multi_buffer.hpp>
#include <boost/beast/http/message.hpp>
#include <boost/beast/http/string_body.hpp>
#include <boost/system/error_code.hpp>

#include "connection_settings.h"
#include "rtc/rtc_manager.h"
#include "util.h"
#include "watchdog.h"
#include "websocket.h"

class P2PWebsocketSession
    : public std::enable_shared_from_this<P2PWebsocketSession>,
      public RTCMessageSender {
  P2PWebsocketSession(boost::asio::io_context& ioc,
                      boost::asio::ip::tcp::socket socket,
                      RTCManager* rtc_manager,
                      ConnectionSettings conn_settings);

 public:
  static std::shared_ptr<P2PWebsocketSession> Create(
      boost::asio::io_context& ioc,
      boost::asio::ip::tcp::socket socket,
      RTCManager* rtc_manager,
      ConnectionSettings conn_settings) {
    return std::shared_ptr<P2PWebsocketSession>(new P2PWebsocketSession(
        ioc, std::move(socket), rtc_manager, conn_settings));
  }
  ~P2PWebsocketSession();

  void Run(boost::beast::http::request<boost::beast::http::string_body> req);

 private:
  void OnWatchdogExpired();
  void DoAccept(
      boost::beast::http::request<boost::beast::http::string_body> req);
  void OnAccept(boost::system::error_code ec);

  void DoRead();
  void OnRead(boost::system::error_code ec,
              std::size_t bytes_transferred,
              std::string recv_string);

  std::shared_ptr<RTCConnection> CreateRTCConnection();

 private:
  //WebRTC
  void onIceConnectionStateChange(
      webrtc::PeerConnectionInterface::IceConnectionState new_state) override;
  void onIceCandidate(const std::string sdp_mid,
                      const int sdp_mlineindex,
                      const std::string sdp) override;

 private:
  std::unique_ptr<Websocket> ws_;

  WatchDog watchdog_;

  RTCManager* rtc_manager_;
  ConnectionSettings conn_settings_;

  std::shared_ptr<RTCConnection> connection_;
  webrtc::PeerConnectionInterface::IceConnectionState rtc_state_;
};

#endif  // P2P_WEBSOCKET_SESSION_H_
