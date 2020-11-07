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

#include "metrics/stats_collector.h"
#include "rtc/rtc_manager.h"
#include "rtc/rtc_message_sender.h"
#include "url_parts.h"
#include "watchdog.h"
#include "websocket.h"

struct AyameClientConfig {
  bool insecure = false;
  bool no_google_stun = false;

  std::string signaling_url;
  std::string room_id;
  std::string client_id;
  std::string signaling_key;
};

class AyameClient : public std::enable_shared_from_this<AyameClient>,
                    public RTCMessageSender,
                    public StatsCollector {
  AyameClient(boost::asio::io_context& ioc,
              RTCManager* manager,
              AyameClientConfig config);

 public:
  static std::shared_ptr<AyameClient> Create(boost::asio::io_context& ioc,
                                             RTCManager* manager,
                                             AyameClientConfig config) {
    return std::shared_ptr<AyameClient>(
        new AyameClient(ioc, manager, std::move(config)));
  }
  ~AyameClient();

  void Reset();
  void Connect();
  void Close();

  void GetStats(std::function<
                void(const rtc::scoped_refptr<const webrtc::RTCStatsReport>&)>
                    callback) override;

 private:
  void ReconnectAfter();
  void OnWatchdogExpired();
  bool ParseURL(URLParts& parts) const;

 private:
  void DoRead();
  void DoRegister();
  void DoSendPong();
  void SetIceServersFromConfig(nlohmann::json json_message);
  void CreatePeerConnection();

 private:
  void OnConnect(boost::system::error_code ec);
  void OnClose(boost::system::error_code ec);
  void OnRead(boost::system::error_code ec,
              std::size_t bytes_transferred,
              std::string text);

 private:
  // WebRTC からのコールバック
  // これらは別スレッドからやってくるので取り扱い注意
  void OnIceConnectionStateChange(
      webrtc::PeerConnectionInterface::IceConnectionState new_state) override;
  void OnIceCandidate(const std::string sdp_mid,
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
  AyameClientConfig config_;

  int retry_count_;
  webrtc::PeerConnectionInterface::IceConnectionState rtc_state_;

  WatchDog watchdog_;

  bool is_send_offer_;
  bool has_is_exist_user_flag_;

  webrtc::PeerConnectionInterface::IceServers ice_servers_;
};

#endif  // AYAME_CLIENT_H_
