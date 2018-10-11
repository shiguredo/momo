#include "sora_connection.h"

SoraConnection::SoraConnection(
        RTCManager *manager, WebSocketClient *wsclient, ConnectionSettings conn_settings) :
        _manager(manager), _wsclient(wsclient), _conn_settings(conn_settings),
        _rtc_state(IceConnectionState::kIceConnectionNew)
{
  _retry_count = 0;
  _watchdog.reset(new WatchDog(std::bind(&SoraConnection::onWatchdogExpired, this)));
}

SoraConnection::~SoraConnection()
{
  disconnect();
}

std::shared_ptr<RTCConnection> SoraConnection::getRTCConnection() {
  if (_rtc_state == IceConnectionState::kIceConnectionConnected) {
    return _connection;
  } else {
    return nullptr;
  }
}

bool SoraConnection::connect()
{
  RTC_LOG(LS_INFO) << __FUNCTION__;
  if (_endpoint) {
    return false;
  }
  _endpoint = _wsclient->createEndpoint(_conn_settings.sora_signaling_host, this);
  _watchdog->enable(30);
  return true;
}

bool SoraConnection::disconnect()
{
  RTC_LOG(LS_INFO) << __FUNCTION__;
  _watchdog->disable();
  _connection = nullptr;
  if (_endpoint) {
    _endpoint->release();
    _endpoint = nullptr;
    return true;
  } else {
    return false;
  }
}

void SoraConnection::reconnect(bool now)
{
  if (now) {
    RTC_LOG(LS_INFO) << __FUNCTION__ << " reconnecting...:";
    _connection = nullptr;
    _endpoint = nullptr;
    connect();
  } else {
    int interval = 5 * (2 * _retry_count + 1);
    RTC_LOG(LS_INFO) << __FUNCTION__ << " reconnect after " << interval <<" sec";
    _watchdog->enable(interval);
    _retry_count++;
  }
}

void SoraConnection::onOpen()
{
  sendConnect();
}

void SoraConnection::onFail()
{
  reconnect(false);
}

void SoraConnection::onMessage(std::string message)
{
  parseSoraMessage(message);
}

void SoraConnection::onWatchdogExpired() {
  RTC_LOG(LS_WARNING) << __FUNCTION__;
  reconnect(true);
}

void SoraConnection::parseSoraMessage(std::string message)
{
  auto json_message = json::parse(message);
  const std::string type = json_message["type"];
  if (type == "offer") {
    createPeerFromConfig(json_message["config"]);
    const std::string sdp = json_message["sdp"];
    _connection->setOffer(sdp);
  } else if (type == "notify") {
  } else if (type == "ping") {
    if (_rtc_state != IceConnectionState::kIceConnectionConnected)
    {
      return;
    }
    _watchdog->reset();
    sendPong();
  }
}

void SoraConnection::createPeerFromConfig(json jconfig)
{
  webrtc::PeerConnectionInterface::RTCConfiguration rtc_config;
  webrtc::PeerConnectionInterface::IceServers ice_servers;

  auto jservers = jconfig["iceServers"];
  for (auto jserver : jservers)
  {
    const std::string username = jserver["username"];
    const std::string credential = jserver["credential"];
    auto jurls = jserver["urls"];
    for (const std::string url : jurls)
    {
      webrtc::PeerConnectionInterface::IceServer ice_server;
      ice_server.uri = url;
      ice_server.username = username;
      ice_server.password = credential;
      ice_servers.push_back(ice_server);
    }
  }

  rtc_config.servers = ice_servers;

  _connection = _manager->createConnection(rtc_config, this);
}

void SoraConnection::sendConnect()
{
  json json_message = {
    {"type", "connect"},
    {"role", "upstream"},
    {"channel_id", _conn_settings.sora_channel_id},
  };

  if (!_conn_settings.metadata.empty()) {
    json_message["metadata"] = _conn_settings.metadata;
  }

  json_message["video"]["codec_type"] = _conn_settings.video_codec;
  if (_conn_settings.video_bitrate != 0) {
    json_message["video"]["bit_rate"] = _conn_settings.video_bitrate;
  }

  json_message["audio"]["codec_type"] = _conn_settings.audio_codec;
  if (_conn_settings.audio_bitrate != 0) {
    json_message["audio"]["bit_rate"] = _conn_settings.audio_bitrate;
  }

  _endpoint->sendText(json_message.dump());
}

void SoraConnection::sendPong()
{
  json json_message = {
    {"type", "pong"}
  };
  _endpoint->sendText(json_message.dump());
}

void SoraConnection::sendAnswer(std::string answer)
{
  json json_message = {
    {"type", "answer"},
    {"sdp", answer}
  };
  _endpoint->sendText(json_message.dump());
}

void SoraConnection::sendCandidate(std::string candidate)
{
  json json_message = {
    {"type", "candidate"},
    {"candidate", candidate}
  };
  _endpoint->sendText(json_message.dump());
}

// Connecting sequence Checking -> Connected
// When line disconnected Disconnected -> Failed
void SoraConnection::onIceConnectionStateChange(IceConnectionState new_state)
{
  RTC_LOG(LS_INFO) << __FUNCTION__ << " state:" << new_state;
  switch (new_state) {
    case IceConnectionState::kIceConnectionConnected:
      _retry_count = 0;
      _watchdog->enable(60);
      break;
    case IceConnectionState::kIceConnectionFailed:
      reconnect(false);
      break;
    default:
      break;
  }
  _rtc_state = new_state;
}

void SoraConnection::onIceCandidate(
        const std::string sdp_mid, const int sdp_mlineindex, const std::string sdp)
{
  sendCandidate(sdp);
}

void SoraConnection::onCreateDescription(webrtc::SdpType type, const std::string sdp)
{
  sendAnswer(sdp);
}
