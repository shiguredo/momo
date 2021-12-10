#ifndef MOMO_ARGS_H_
#define MOMO_ARGS_H_

#include <iostream>
#include <string>

// Boost
#include <boost/json.hpp>

// WebRTC
#include <api/rtp_parameters.h>

#include "video_codec_info.h"

struct MomoArgs {
  bool no_google_stun = false;
  bool no_video_device = false;
  bool no_audio_device = false;
  bool force_i420 = false;
  // Jetson の場合だけデフォルト true
#if USE_JETSON_ENCODER
  bool hw_mjpeg_decoder = true;
#else
  bool hw_mjpeg_decoder = false;
#endif
  std::string video_device = "";
  std::string resolution = "VGA";
  int framerate = 30;
  bool fixed_resolution = false;
  std::string priority = "FRAMERATE";
  bool use_sdl = false;
  bool show_me = false;
  int window_width = 640;
  int window_height = 480;
  bool fullscreen = false;
  std::string serial_device = "";
  unsigned int serial_rate = 9600;
  bool insecure = false;
  bool screen_capture = false;
  int metrics_port = -1;
  bool metrics_allow_external_ip = false;

  std::vector<std::string> sora_signaling_urls;
  std::string sora_channel_id;
  bool sora_video = true;
  bool sora_audio = true;
  // 空文字の場合コーデックは Sora 側で決める
  std::string sora_video_codec_type = "";
  std::string sora_audio_codec_type = "";
  // 0 の場合ビットレートは Sora 側で決める
  int sora_video_bit_rate = 0;
  int sora_audio_bit_rate = 0;
  bool sora_auto_connect = false;
  boost::json::value sora_metadata;
  // upstream or downstream
  std::string sora_role = "upstream";
  bool sora_multistream = false;
  bool sora_spotlight = false;
  int sora_spotlight_number = 0;
  int sora_port = -1;
  bool sora_simulcast = false;
  boost::optional<bool> sora_data_channel_signaling;
  int sora_data_channel_signaling_timeout = 180;
  boost::optional<bool> sora_ignore_disconnect_websocket;
  int sora_disconnect_wait_timeout = 5;

  std::string test_document_root;
  int test_port = 8080;

  std::string ayame_signaling_url;
  std::string ayame_room_id;
  std::string ayame_client_id = "";
  std::string ayame_signaling_key = "";

  bool disable_echo_cancellation = false;
  bool disable_auto_gain_control = false;
  bool disable_noise_suppression = false;
  bool disable_highpass_filter = false;
  bool disable_typing_detection = false;
  bool disable_residual_echo_detector = false;

  VideoCodecInfo::Type vp8_encoder = VideoCodecInfo::Type::Default;
  VideoCodecInfo::Type vp8_decoder = VideoCodecInfo::Type::Default;
  VideoCodecInfo::Type vp9_encoder = VideoCodecInfo::Type::Default;
  VideoCodecInfo::Type vp9_decoder = VideoCodecInfo::Type::Default;
  VideoCodecInfo::Type av1_encoder = VideoCodecInfo::Type::Default;
  VideoCodecInfo::Type av1_decoder = VideoCodecInfo::Type::Default;
  VideoCodecInfo::Type h264_encoder = VideoCodecInfo::Type::Default;
  VideoCodecInfo::Type h264_decoder = VideoCodecInfo::Type::Default;

  struct Size {
    int width;
    int height;
  };
  Size GetSize() {
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
};

#endif
