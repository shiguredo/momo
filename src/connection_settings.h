#ifndef CONNECTION_SETTINGS_H_
#define CONNECTION_SETTINGS_H_

#include <string>
#include <iostream>
#include <nlohmann/json.hpp>

#include "api/rtpparameters.h"

struct ConnectionSettings
{
  bool no_video = false;
  bool no_audio = false;
  std::string video_codec = "VP8";
  std::string audio_codec = "OPUS";
  int video_bitrate = 0;
  int audio_bitrate = 0;
  std::string resolution = "VGA";
  int framerate = 0;
  std::string priority = "BALANCE";

  std::string sora_signaling_host = "wss://example.com/signaling";
  std::string sora_channel_id;
  bool sora_auto_connect = false;
  nlohmann::json sora_metadata;

  int p2p_port = 8080;
  std::string p2p_document_root;

  int getWidth() {
    if (resolution == "QVGA") {
      return 480;
    } else if (resolution == "HD") {
      return 1280;
    } else if (resolution == "FHD") {
      return 1920;
    } else if (resolution == "4K") {
      return 3840;
    }
    return 640;
  }

  int getHeight() {
    if (resolution == "QVGA") {
      return 320;
    } else if (resolution == "HD") {
      return 720;
    } else if (resolution == "FHD") {
      return 1080;
    } else if (resolution == "4K") {
      return 2160;
    }
    return 480;
  }

  // FRAMERATE が優先のときは RESOLUTION をデグレさせていく
  webrtc::DegradationPreference getPriority() {
    if (priority == "FRAMERATE") {
      return webrtc::DegradationPreference::MAINTAIN_RESOLUTION;
    } else if (priority == "RESOLUTION") {
      return webrtc::DegradationPreference::MAINTAIN_FRAMERATE;
    }
    return webrtc::DegradationPreference::BALANCED;
  }

  friend std::ostream& operator<<(std::ostream& os, const ConnectionSettings& cs) {
    os << "no_video: " << (cs.no_video ? "true" : "false") << "\n";
    os << "no_audio: " << (cs.no_audio ? "true" : "false") << "\n";
    os << "video_codec: " << cs.video_codec << "\n";
    os << "audio_codec: " << cs.audio_codec << "\n";
    os << "video_bitrate: " << cs.video_bitrate << "\n";
    os << "audio_bitrate: " << cs.audio_bitrate << "\n";
    os << "resolution: " << cs.resolution << "\n";
    os << "framerate: " << cs.framerate << "\n";
    os << "priority: " << cs.priority << "\n";
    os << "sora_signaling_host: " << cs.sora_signaling_host << "\n";
    os << "sora_channel_id: " << cs.sora_channel_id << "\n";
    os << "sora_auto_connect: " << (cs.sora_auto_connect ? "true" : "false") << "\n";
    os << "sora_metadata: " << cs.sora_metadata << "\n";
    os << "p2p_port: " << cs.p2p_port << "\n";
    os << "p2p_document_root: " << cs.p2p_document_root << "\n";
    return os;
  }
};

#endif
