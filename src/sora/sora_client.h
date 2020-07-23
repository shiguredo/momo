#ifndef SORA_CLIENT_H_
#define SORA_CLIENT_H_

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

class SoraClient : public std::enable_shared_from_this<SoraClient>,
                   public RTCMessageSender {
 public:
  SoraClient(boost::asio::io_context& ioc,
             RTCManager* manager,
             ConnectionSettings conn_settings);
  ~SoraClient();
  void Reset();
  bool Connect();
  void Close();

  webrtc::PeerConnectionInterface::IceConnectionState GetRTCConnectionState()
      const;
  std::shared_ptr<RTCConnection> GetRTCConnection() const;

 private:
  void ReconnectAfter();
  void OnWatchdogExpired();

 private:
  bool ParseURL(URLParts& parts) const;
  boost::asio::ssl::context CreateSSLContext() const;

 private:
  void OnResolve(boost::system::error_code ec,
                 boost::asio::ip::tcp::resolver::results_type results);
  void OnSSLConnect(boost::system::error_code ec);
  void OnSSLHandshake(boost::system::error_code ec);
  void OnConnect(boost::system::error_code ec);
  void OnHandshake(boost::system::error_code ec);
  void DoSendConnect();
  void DoSendPong();
  void DoSendPong(
      const rtc::scoped_refptr<const webrtc::RTCStatsReport>& report);
  void CreatePeerFromConfig(nlohmann::json jconfig);

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
};

#endif  // SORA_CLIENT_H_
