#ifndef AYAME_CLIENT_H_
#define AYAME_CLIENT_H_

#include <algorithm>
#include <cstdlib>
#include <functional>
#include <memory>
#include <string>

// Boost
#include <boost/asio/io_context.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/ssl/context.hpp>

// nlohmann/json
#include <nlohmann/json.hpp>

#include "connection_settings.h"
#include "rtc/rtc_manager.h"
#include "rtc/rtc_message_sender.h"
#include "url_parts.h"
#include "watchdog.h"
#include "ws/websocket.h"

class AyameClient : public std::enable_shared_from_this<AyameClient>,
                    public RTCMessageSender {
 public:
  AyameClient(boost::asio::io_context& ioc,
              RTCManager* manager,
              ConnectionSettings conn_settings);
  ~AyameClient();
  void Reset();
  bool Connect();
  void Close();

  bool ParseURL(URLParts& parts) const;
  boost::asio::ssl::context CreateSSLContext() const;

 private:
  void ReconnectAfter();
  void OnWatchdogExpired();

 private:
  void OnResolve(boost::system::error_code ec,
                 boost::asio::ip::tcp::resolver::results_type results);
  void OnSSLConnect(boost::system::error_code ec);
  void OnSSLHandshake(boost::system::error_code ec);
  void OnConnect(boost::system::error_code ec);
  void OnHandshake(boost::system::error_code ec);
  void DoRegister();
  void DoSendPong();
  void SetIceServersFromConfig(nlohmann::json json_message);
  void CreatePeerConnection();

 private:
  void OnClose(boost::system::error_code ec);
  void OnRead(boost::system::error_code ec,
              std::size_t bytes_transferred,
              std::string text);

 private:
  // WebRTC からのコールバック
  // これらは別スレッドからやってくるので取り扱い注意
  void onIceConnectionStateChange(
      webrtc::PeerConnectionInterface::IceConnectionState new_state) override;
  void onIceCandidate(const std::string sdp_mid,
                      const int sdp_mlineindex,
                      const std::string sdp) override;

 private:
  void DoIceConnectionStateChange(
      webrtc::PeerConnectionInterface::IceConnectionState new_state);

 private:
  boost::asio::io_context& ioc_;

  boost::asio::ip::tcp::resolver resolver_;
  std::unique_ptr<Websocket> ws_;
  URLParts parts_;

  std::atomic_bool destructed_ = {false};

  RTCManager* manager_;
  std::shared_ptr<RTCConnection> connection_;
  ConnectionSettings conn_settings_;

  int retry_count_;
  webrtc::PeerConnectionInterface::IceConnectionState rtc_state_;

  WatchDog watchdog_;

  bool is_send_offer_;
  bool has_is_exist_user_flag_;

  webrtc::PeerConnectionInterface::IceServers ice_servers_;
};

#endif  // AYAME_CLIENT_H_