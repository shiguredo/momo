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
#include "websocket.h"

class SoraClient : public std::enable_shared_from_this<SoraClient>,
                   public RTCMessageSender {
  SoraClient(boost::asio::io_context& ioc,
             RTCManager* manager,
             ConnectionSettings conn_settings);

 public:
  static std::shared_ptr<SoraClient> Create(boost::asio::io_context& ioc,
                                            RTCManager* manager,
                                            ConnectionSettings conn_settings) {
    return std::shared_ptr<SoraClient>(
        new SoraClient(ioc, manager, conn_settings));
  }
  ~SoraClient();

  void Reset();
  void Connect();
  void Close();

  webrtc::PeerConnectionInterface::IceConnectionState GetRTCConnectionState()
      const;
  std::shared_ptr<RTCConnection> GetRTCConnection() const;

 private:
  void ReconnectAfter();
  void OnWatchdogExpired();
  bool ParseURL(URLParts& parts) const;

 private:
  void DoRead();
  void DoSendConnect();
  void DoSendPong();
  void DoSendPong(
      const rtc::scoped_refptr<const webrtc::RTCStatsReport>& report);
  void CreatePeerFromConfig(nlohmann::json jconfig);

 private:
  void OnConnect(boost::system::error_code ec);
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
  std::unique_ptr<Websocket> ws_;

  std::atomic_bool destructed_ = {false};

  RTCManager* manager_;
  std::shared_ptr<RTCConnection> connection_;
  ConnectionSettings conn_settings_;

  int retry_count_;
  webrtc::PeerConnectionInterface::IceConnectionState rtc_state_;

  WatchDog watchdog_;
};

#endif  // SORA_CLIENT_H_
