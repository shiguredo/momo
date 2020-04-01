#ifndef CONNECTION_SETTINGS_H_
#define CONNECTION_SETTINGS_H_

#include <iostream>
#include <nlohmann/json.hpp>
#include <string>

#include "api/rtp_parameters.h"

struct ConnectionSettings {
  std::string camera_name = "";
#if USE_ROS
  bool image_compressed = false;
  std::string audio_topic_name = "";
  int audio_topic_rate = 16000;
  int audio_topic_ch = 1;
#endif

  bool no_video_device = false;
  bool no_audio_device = false;
  bool force_i420 = false;
  bool use_native = false;
  std::string video_device = "";
  std::string resolution = "VGA";
  int framerate = 30;
  bool fixed_resolution = false;
  std::string priority = "BALANCE";
  bool use_sdl = false;
  bool show_me = false;
  int window_width = 640;
  int window_height = 480;
  bool fullscreen = false;
  std::string serial_device = "";
  unsigned int serial_rate = 9600;
  bool insecure = false;

  std::string sora_signaling_host = "wss://example.com/signaling";
  std::string sora_channel_id;
  bool sora_video = true;
  bool sora_audio = true;
  // 空文字の場合コーデックは Sora 側で決める
  std::string sora_video_codec = "";
  std::string sora_audio_codec = "";
  // 0 の場合ビットレートは Sora 側で決める
  int sora_video_bitrate = 0;
  int sora_audio_bitrate = 0;
  bool sora_auto_connect = false;
  nlohmann::json sora_metadata;
  // upstream or downstream
  std::string sora_role = "upstream";
  bool sora_multistream = false;
  int sora_spotlight = -1;
  int sora_port = -1;

  std::string test_document_root;
  int test_port = 8080;

  std::string ayame_signaling_host;
  std::string ayame_room_id;
  std::string ayame_client_id = "";
  std::string ayame_signaling_key = "";

  bool disable_echo_cancellation = false;
  bool disable_auto_gain_control = false;
  bool disable_noise_suppression = false;
  bool disable_highpass_filter = false;
  bool disable_typing_detection = false;
  bool disable_residual_echo_detector = false;

  struct Size {
    int width;
    int height;
  };
  Size getSize() {
    if (resolution == "QVGA") {
      return {320, 240};
    } else if (resolution == "VGA") {
      return {640, 480};
    } else if (resolution == "HD") {
      return {1280, 720};
    } else if (resolution == "FHD") {
      return {1920, 1080};
    } else if (resolution == "4K") {
      return {3840, 2160};
    }

    // 128x96 みたいな感じのフォーマット
    auto pos = resolution.find('x');
    if (pos == std::string::npos) {
      return {16, 16};
    }
    auto width = std::atoi(resolution.substr(0, pos).c_str());
    auto height = std::atoi(resolution.substr(pos + 1).c_str());
    return {std::max(16, width), std::max(16, height)};
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

  friend std::ostream& operator<<(std::ostream& os,
                                  const ConnectionSettings& cs) {
    os << "no_video_device: " << (cs.no_video_device ? "true" : "false")
       << "\n";
    os << "no_audio_device: " << (cs.no_audio_device ? "true" : "false")
       << "\n";
    os << "resolution: " << cs.resolution << "\n";
    os << "framerate: " << cs.framerate << "\n";
    os << "fixed_resolution: " << (cs.fixed_resolution ? "true" : "false")
       << "\n";
    os << "priority: " << cs.priority << "\n";
    os << "ayame_signaling_host: " << cs.ayame_signaling_host << "\n";
    os << "ayame_room_id: " << cs.ayame_room_id << "\n";
    os << "ayame_client_id: " << cs.ayame_client_id << "\n";
    os << "sora_signaling_host: " << cs.sora_signaling_host << "\n";
    os << "sora_channel_id: " << cs.sora_channel_id << "\n";
    os << "sora_video_codec: " << cs.sora_video_codec << "\n";
    os << "sora_audio_codec: " << cs.sora_audio_codec << "\n";
    os << "sora_video_bitrate: " << cs.sora_video_bitrate << "\n";
    os << "sora_audio_bitrate: " << cs.sora_audio_bitrate << "\n";
    os << "sora_auto_connect: " << (cs.sora_auto_connect ? "true" : "false")
       << "\n";
    os << "sora_metadata: " << cs.sora_metadata << "\n";
    os << "sora_port: " << cs.sora_port << "\n";
    os << "test_document_root: " << cs.test_document_root << "\n";
    os << "test_port: " << cs.test_port << "\n";
    return os;
  }
};

#endif
