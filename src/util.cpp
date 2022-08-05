#include "util.h"

#include <regex>

// CLI11
#include <CLI/CLI.hpp>

// Boost
#include <boost/beast/version.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/json.hpp>
#include <boost/optional/optional_io.hpp>
#include <boost/preprocessor/stringize.hpp>

// WebRTC
#include <rtc_base/helpers.h>

#include "momo_version.h"

void Util::ParseArgs(int argc,
                     char* argv[],
                     bool& use_test,
                     bool& use_ayame,
                     bool& use_sora,
                     int& log_level,
                     MomoArgs& args) {
  CLI::App app("Momo - WebRTC Native Client");
  app.set_help_all_flag("--help-all",
                        "Print help message for all modes and exit");

  bool version = false;
  bool video_codecs = false;

  auto is_valid_force_i420 = CLI::Validator(
      [](std::string input) -> std::string {
#if USE_MMAL_ENCODER || USE_JETSON_ENCODER || USE_NVCODEC_ENCODER
        return std::string();
#else
        return "Not available because your device does not have this feature.";
#endif
      },
      "");
  auto is_valid_hw_mjpeg_decoder = CLI::Validator(
      [](std::string input) -> std::string {
        if (input == "1") {
#if USE_MMAL_ENCODER || USE_JETSON_ENCODER || USE_NVCODEC_ENCODER
          return std::string();
#else
          return "Not available because your device does not have this "
                 "feature.";
#endif
        }
        return std::string();
      },
      "");

  auto is_valid_h264 = CLI::Validator(
      [](std::string input) -> std::string {
#if USE_H264
        return std::string();
#else
        if (input == "H264") {
          return "Not available because your device does not have this "
                 "feature.";
        }
        return std::string();
#endif
      },
      "");

  auto is_sdl_available = CLI::Validator(
      [](std::string input) -> std::string {
#if USE_SDL2
        return std::string();
#else
        return "Not available because your device does not have this "
               "feature.";
#endif
      },
      "");

  auto is_valid_resolution = CLI::Validator(
      [](std::string input) -> std::string {
        if (input == "QVGA" || input == "VGA" || input == "HD" ||
            input == "FHD" || input == "4K") {
          return std::string();
        }

        // 数値x数値、というフォーマットになっているか確認する
        std::regex re("^[1-9][0-9]*x[1-9][0-9]*$");
        if (std::regex_match(input, re)) {
          return std::string();
        }

        return "Must be one of QVGA, VGA, HD, FHD, 4K, or "
               "[WIDTH]x[HEIGHT].";
      },
      "");

  auto is_valid_screen_capture = CLI::Validator(
      [](std::string input) -> std::string {
#if USE_SCREEN_CAPTURER
        return std::string();
#else
        return "Not available because your device does not have this feature.";
#endif
      },
      "");

  auto bool_map = std::vector<std::pair<std::string, bool>>(
      {{"false", false}, {"true", true}});
  auto optional_bool_map =
      std::vector<std::pair<std::string, boost::optional<bool>>>(
          {{"false", false}, {"true", true}, {"none", boost::none}});

  app.add_flag("--no-google-stun", args.no_google_stun,
               "Do not use google stun");
  app.add_flag("--no-video-device", args.no_video_device,
               "Do not use video device");
  app.add_flag("--no-audio-device", args.no_audio_device,
               "Do not use audio device");
  app.add_flag(
         "--force-i420", args.force_i420,
         "Prefer I420 format for video capture (only on supported devices)")
      ->check(is_valid_force_i420);
  app.add_option(
         "--hw-mjpeg-decoder", args.hw_mjpeg_decoder,
         "Perform MJPEG deoode and video resize by hardware acceleration "
         "(only on supported devices)")
      ->check(is_valid_hw_mjpeg_decoder)
      ->transform(CLI::CheckedTransformer(bool_map, CLI::ignore_case));
#if defined(__APPLE__) || defined(_WIN32)
  app.add_option("--video-device", args.video_device,
                 "Use the video device specified by an index or a name "
                 "(use the first one if not specified)");
#elif defined(__linux__)
  app.add_option("--video-device", args.video_device,
                 "Use the video input device specified by a name "
                 "(some device will be used if not specified)")
      ->check(CLI::ExistingFile);
#endif
  app.add_option("--resolution", args.resolution,
                 "Video resolution (one of QVGA, VGA, HD, FHD, 4K, or "
                 "[WIDTH]x[HEIGHT])")
      ->check(is_valid_resolution);
  app.add_option("--framerate", args.framerate, "Video framerate")
      ->check(CLI::Range(1, 60));
  app.add_flag("--fixed-resolution", args.fixed_resolution,
               "Maintain video resolution in degradation");
  app.add_option(
         "--priority", args.priority,
         "Specifies the quality that is maintained against video degradation")
      ->check(CLI::IsMember({"BALANCE", "FRAMERATE", "RESOLUTION"}));
  app.add_flag("--use-sdl", args.use_sdl,
               "Show video using SDL (if SDL is available)")
      ->check(is_sdl_available);
  app.add_flag("--show-me", args.show_me,
               "Show self video (if SDL is available)")
      ->check(is_sdl_available);
  app.add_option("--window-width", args.window_width,
                 "Window width for videos (if SDL is available)")
      ->check(is_sdl_available)
      ->check(CLI::Range(180, 16384));
  app.add_option("--window-height", args.window_height,
                 "Window height for videos (if SDL is available)")
      ->check(is_sdl_available)
      ->check(CLI::Range(180, 16384));
  app.add_flag("--fullscreen", args.fullscreen,
               "Use fullscreen window for videos (if SDL is available)")
      ->check(is_sdl_available);
  app.add_flag("--version", version, "Show version information");
  app.add_flag("--insecure", args.insecure,
               "Allow insecure server connections when using SSL");
  auto log_level_map = std::vector<std::pair<std::string, int>>(
      {{"verbose", 0}, {"info", 1}, {"warning", 2}, {"error", 3}, {"none", 4}});
  app.add_option("--log-level", log_level, "Log severity level threshold")
      ->transform(CLI::CheckedTransformer(log_level_map, CLI::ignore_case));

  app.add_flag("--screen-capture", args.screen_capture, "Capture screen")
      ->check(is_valid_screen_capture);

  // オーディオフラグ
  app.add_flag("--disable-echo-cancellation", args.disable_echo_cancellation,
               "Disable echo cancellation for audio");
  app.add_flag("--disable-auto-gain-control", args.disable_auto_gain_control,
               "Disable auto gain control for audio");
  app.add_flag("--disable-noise-suppression", args.disable_noise_suppression,
               "Disable noise suppression for audio");
  app.add_flag("--disable-highpass-filter", args.disable_highpass_filter,
               "Disable highpass filter for audio");

  // ビデオエンコーダ
  app.add_flag("--video-codec-engines", video_codecs,
               "List available video encoders/decoders");
  {
    auto info = VideoCodecInfo::Get();
    // 長いので短くする
    auto f = [](auto x) {
      return CLI::CheckedTransformer(VideoCodecInfo::GetValidMappingInfo(x),
                                     CLI::ignore_case);
    };
    app.add_option("--vp8-encoder", args.vp8_encoder, "VP8 Encoder")
        ->transform(f(info.vp8_encoders));
    app.add_option("--vp8-decoder", args.vp8_decoder, "VP8 Decoder")
        ->transform(f(info.vp8_decoders));
    app.add_option("--vp9-encoder", args.vp9_encoder, "VP9 Encoder")
        ->transform(f(info.vp9_encoders));
    app.add_option("--vp9-decoder", args.vp9_decoder, "VP9 Decoder")
        ->transform(f(info.vp9_decoders));
    app.add_option("--av1-encoder", args.av1_encoder, "AV1 Encoder")
        ->transform(f(info.av1_encoders));
    app.add_option("--av1-decoder", args.av1_decoder, "AV1 Decoder")
        ->transform(f(info.av1_decoders));
    app.add_option("--h264-encoder", args.h264_encoder, "H.264 Encoder")
        ->transform(f(info.h264_encoders));
    app.add_option("--h264-decoder", args.h264_decoder, "H.264 Decoder")
        ->transform(f(info.h264_decoders));
  }

  auto is_serial_setting_format = CLI::Validator(
      [](std::string input) -> std::string {
        try {
          auto separater_pos = input.find(',');
          std::string baudrate_str = input.substr(separater_pos + 1);
          unsigned int _ = std::stoi(baudrate_str);
          return std::string();
        } catch (std::invalid_argument& e) {
          return "Value " + input +
                 " is not serial setting format [DEVICE],[BAUDRATE]";
        } catch (std::out_of_range& e) {
          return "Value " + input +
                 " is not serial setting format [DEVICE],[BAUDRATE]";
        }
      },
      "serial setting format");
  std::string serial_setting;
  app.add_option(
         "--serial", serial_setting,
         "Serial port settings for datachannel passthrough [DEVICE],[BAUDRATE]")
      ->check(is_serial_setting_format);

  app.add_option("--metrics-port", args.metrics_port,
                 "Metrics server port number (default: -1)")
      ->check(CLI::Range(-1, 65535));
  app.add_flag("--metrics-allow-external-ip", args.metrics_allow_external_ip,
               "Allow access to Metrics server from external IP");

  app.add_option("--client-cert", args.client_cert,
                 "Cert file path for client certification (PEM format)")
      ->check(CLI::ExistingFile);
  app.add_option("--client-key", args.client_key,
                 "Private key file path for client certification (PEM format)")
      ->check(CLI::ExistingFile);

  // proxy サーバーの設定
  app.add_option("--proxy-url", args.proxy_url, "Proxy URL");
  app.add_option("--proxy-username", args.proxy_username, "Proxy username");
  app.add_option("--proxy-password", args.proxy_password, "Proxy password");

  auto test_app = app.add_subcommand(
      "test", "Mode for momo development with simple HTTP server");
  auto ayame_app = app.add_subcommand(
      "ayame", "Mode for working with WebRTC Signaling Server Ayame");
  auto sora_app =
      app.add_subcommand("sora", "Mode for working with WebRTC SFU Sora");

  test_app
      ->add_option("--document-root", args.test_document_root,
                   "HTTP document root directory")
      ->check(CLI::ExistingDirectory);
  test_app->add_option("--port", args.test_port, "Port number (default: 8080)")
      ->check(CLI::Range(0, 65535));

  ayame_app
      ->add_option("--signaling-url", args.ayame_signaling_url, "Signaling URL")
      ->required();
  ayame_app->add_option("--channel-id", args.ayame_room_id, "Channel ID")
      ->required();
  ayame_app->add_option("--client-id", args.ayame_client_id, "Client ID");
  ayame_app->add_option("--signaling-key", args.ayame_signaling_key,
                        "Signaling key");

  sora_app
      ->add_option("--signaling-url", args.sora_signaling_urls,
                   "Signaling URLs")
      ->take_all()
      ->required();
  sora_app->add_option("--channel-id", args.sora_channel_id, "Channel ID")
      ->required();
  sora_app->add_flag("--auto", args.sora_auto_connect,
                     "Connect to Sora automatically");

  sora_app
      ->add_option("--video", args.sora_video,
                   "Send video to sora (default: true)")
      ->transform(CLI::CheckedTransformer(bool_map, CLI::ignore_case));
  sora_app
      ->add_option("--audio", args.sora_audio,
                   "Send audio to sora (default: true)")
      ->transform(CLI::CheckedTransformer(bool_map, CLI::ignore_case));
  sora_app
      ->add_option("--video-codec-type", args.sora_video_codec_type,
                   "Video codec for send")
      ->check(CLI::IsMember({"", "VP8", "VP9", "AV1", "H264"}))
      ->check(is_valid_h264);
  sora_app
      ->add_option("--audio-codec-type", args.sora_audio_codec_type,
                   "Audio codec for send")
      ->check(CLI::IsMember({"", "OPUS"}));
  sora_app
      ->add_option("--video-bit-rate", args.sora_video_bit_rate,
                   "Video bit rate")
      ->check(CLI::Range(0, 30000));
  sora_app
      ->add_option("--audio-bit-rate", args.sora_audio_bit_rate,
                   "Audio bit rate")
      ->check(CLI::Range(0, 510));
  sora_app
      ->add_option("--multistream", args.sora_multistream,
                   "Use multistream (default: true)")
      ->transform(CLI::CheckedTransformer(bool_map, CLI::ignore_case));
  sora_app->add_option("--role", args.sora_role, "Role (default: sendonly)")
      ->check(CLI::IsMember({"sendonly", "recvonly", "sendrecv"}));
  sora_app->add_option("--spotlight", args.sora_spotlight, "Use spotlight")
      ->transform(CLI::CheckedTransformer(bool_map, CLI::ignore_case));
  sora_app
      ->add_option("--spotlight-number", args.sora_spotlight_number,
                   "Stream count delivered in spotlight")
      ->check(CLI::Range(0, 8));
  sora_app->add_option("--port", args.sora_port, "Port number (default: -1)")
      ->check(CLI::Range(-1, 65535));
  sora_app
      ->add_option("--simulcast", args.sora_simulcast,
                   "Use simulcast (default: false)")
      ->transform(CLI::CheckedTransformer(bool_map, CLI::ignore_case));
  sora_app
      ->add_option("--data-channel-signaling", args.sora_data_channel_signaling,
                   "Use DataChannel for Sora signaling (default: none)")
      ->type_name("TEXT")
      ->transform(CLI::CheckedTransformer(optional_bool_map, CLI::ignore_case));
  sora_app
      ->add_option("--data-channel-signaling-timeout",
                   args.sora_data_channel_signaling_timeout,
                   "Timeout for Data Channel in seconds (default: 180)")
      ->check(CLI::PositiveNumber);
  sora_app
      ->add_option("--ignore-disconnect-websocket",
                   args.sora_ignore_disconnect_websocket,
                   "Ignore WebSocket disconnection if using Data Channel "
                   "(default: none)")
      ->type_name("TEXT")
      ->transform(CLI::CheckedTransformer(optional_bool_map, CLI::ignore_case));
  sora_app
      ->add_option(
          "--disconnect-wait-timeout", args.sora_disconnect_wait_timeout,
          "Disconnecting timeout for Data Channel in seconds (default: 5)")
      ->check(CLI::PositiveNumber);

  auto is_json = CLI::Validator(
      [](std::string input) -> std::string {
        boost::json::error_code ec;
        boost::json::parse(input);
        if (ec) {
          return "Value " + input + " is not JSON Value";
        }
        return std::string();
      },
      "JSON Value");
  std::string sora_metadata;
  sora_app
      ->add_option("--metadata", sora_metadata,
                   "Signaling metadata used in connect message")
      ->check(is_json);

  try {
    app.parse(argc, argv);
  } catch (const CLI::ParseError& e) {
    exit(app.exit(e));
  }

  // サイマルキャストは VP8, H264 のみで動作する
  if (args.sora_simulcast && args.sora_video_codec_type != "VP8" &&
      args.sora_video_codec_type != "H264") {
    std::cerr << "Simulcast works only --video-codec-type=VP8 or "
                 "--video-codec-type=H264."
              << std::endl;
    exit(1);
  }

  if (!serial_setting.empty()) {
    auto separater_pos = serial_setting.find(',');
    std::string baudrate_str = serial_setting.substr(separater_pos + 1);
    args.serial_device = serial_setting.substr(0, separater_pos);
    args.serial_rate = std::stoi(baudrate_str);
  }

  // メタデータのパース
  if (!sora_metadata.empty()) {
    args.sora_metadata = boost::json::parse(sora_metadata);
  }

  if (args.test_document_root.empty()) {
    args.test_document_root = boost::filesystem::current_path().string();
  }

  if (version) {
    std::cout << MomoVersion::GetClientName() << std::endl;
    std::cout << std::endl;
    std::cout << "WebRTC: " << MomoVersion::GetLibwebrtcName() << std::endl;
    std::cout << "Environment: " << MomoVersion::GetEnvironmentName()
              << std::endl;
    std::cout << std::endl;
    std::cout << "USE_MMAL_ENCODER=" BOOST_PP_STRINGIZE(USE_MMAL_ENCODER)
                                                        << std::endl;
    std::cout << "USE_JETSON_ENCODER=" BOOST_PP_STRINGIZE(USE_JETSON_ENCODER)
                                                          << std::endl;
    std::cout << "USE_NVCODEC_ENCODER=" BOOST_PP_STRINGIZE(USE_NVCODEC_ENCODER)
                                                           << std::endl;
    std::cout << "USE_SDL2=" BOOST_PP_STRINGIZE(USE_SDL2) << std::endl;
    exit(0);
  }

  if (video_codecs) {
    ShowVideoCodecs(VideoCodecInfo::Get());
    exit(0);
  }

  if (!test_app->parsed() && !sora_app->parsed() && !ayame_app->parsed()) {
    std::cout << app.help() << std::endl;
    exit(1);
  }

  if (sora_app->parsed()) {
    use_sora = true;
  }

  if (test_app->parsed()) {
    use_test = true;
  }

  if (ayame_app->parsed()) {
    use_ayame = true;
  }
}

void Util::ShowVideoCodecs(VideoCodecInfo info) {
  // VP8:
  //   Encoder:
  //     - Software (default)
  //   Decoder:
  //     - Jetson (default)
  //     - Software
  //
  // VP9:
  //   Encoder:
  //     - Software (default)
  // ...
  //
  // みたいな感じに出力する
  auto list_codecs = [](std::vector<VideoCodecInfo::Type> types) {
    if (types.empty()) {
      std::cout << "    *UNAVAILABLE*" << std::endl;
      return;
    }

    for (int i = 0; i < types.size(); i++) {
      auto type = types[i];
      auto p = VideoCodecInfo::TypeToString(type);
      std::cout << "    - " << p.first << " [" << p.second << "]";
      if (i == 0) {
        std::cout << " (default)";
      }
      std::cout << std::endl;
    }
  };
  std::cout << "VP8:" << std::endl;
  std::cout << "  Encoder:" << std::endl;
  list_codecs(info.vp8_encoders);
  std::cout << "  Decoder:" << std::endl;
  list_codecs(info.vp8_decoders);
  std::cout << "" << std::endl;
  std::cout << "VP9:" << std::endl;
  std::cout << "  Encoder:" << std::endl;
  list_codecs(info.vp9_encoders);
  std::cout << "  Decoder:" << std::endl;
  list_codecs(info.vp9_decoders);
  std::cout << "" << std::endl;
  std::cout << "AV1:" << std::endl;
  std::cout << "  Encoder:" << std::endl;
  list_codecs(info.av1_encoders);
  std::cout << "  Decoder:" << std::endl;
  list_codecs(info.av1_decoders);
  std::cout << "" << std::endl;
  std::cout << "H264:" << std::endl;
  std::cout << "  Encoder:" << std::endl;
  list_codecs(info.h264_encoders);
  std::cout << "  Decoder:" << std::endl;
  list_codecs(info.h264_decoders);
}

std::string Util::GenerateRandomChars() {
  return GenerateRandomChars(32);
}

std::string Util::GenerateRandomChars(size_t length) {
  std::string result;
  rtc::CreateRandomString(length, &result);
  return result;
}

std::string Util::GenerateRandomNumericChars(size_t length) {
  auto random_numerics = []() -> char {
    const char charset[] = "0123456789";
    const size_t max_index = (sizeof(charset) - 1);
    return charset[rand() % max_index];
  };
  std::string result(length, 0);
  std::generate_n(result.begin(), length, random_numerics);
  return result;
}

std::string Util::IceConnectionStateToString(
    webrtc::PeerConnectionInterface::IceConnectionState state) {
  switch (state) {
    case webrtc::PeerConnectionInterface::kIceConnectionNew:
      return "new";
    case webrtc::PeerConnectionInterface::kIceConnectionChecking:
      return "checking";
    case webrtc::PeerConnectionInterface::kIceConnectionConnected:
      return "connected";
    case webrtc::PeerConnectionInterface::kIceConnectionCompleted:
      return "completed";
    case webrtc::PeerConnectionInterface::kIceConnectionFailed:
      return "failed";
    case webrtc::PeerConnectionInterface::kIceConnectionDisconnected:
      return "disconnected";
    case webrtc::PeerConnectionInterface::kIceConnectionClosed:
      return "closed";
    case webrtc::PeerConnectionInterface::kIceConnectionMax:
      return "max";
  }
  return "unknown";
}

namespace http = boost::beast::http;
using string_view = boost::beast::string_view;

string_view Util::MimeType(string_view path) {
  using boost::beast::iequals;
  auto const ext = [&path] {
    auto const pos = path.rfind(".");
    if (pos == string_view::npos)
      return string_view{};
    return path.substr(pos);
  }();

  if (iequals(ext, ".htm"))
    return "text/html";
  if (iequals(ext, ".html"))
    return "text/html";
  if (iequals(ext, ".php"))
    return "text/html";
  if (iequals(ext, ".css"))
    return "text/css";
  if (iequals(ext, ".txt"))
    return "text/plain";
  if (iequals(ext, ".js"))
    return "application/javascript";
  if (iequals(ext, ".json"))
    return "application/json";
  if (iequals(ext, ".xml"))
    return "application/xml";
  if (iequals(ext, ".swf"))
    return "application/x-shockwave-flash";
  if (iequals(ext, ".flv"))
    return "video/x-flv";
  if (iequals(ext, ".png"))
    return "image/png";
  if (iequals(ext, ".jpe"))
    return "image/jpeg";
  if (iequals(ext, ".jpeg"))
    return "image/jpeg";
  if (iequals(ext, ".jpg"))
    return "image/jpeg";
  if (iequals(ext, ".gif"))
    return "image/gif";
  if (iequals(ext, ".bmp"))
    return "image/bmp";
  if (iequals(ext, ".ico"))
    return "image/vnd.microsoft.icon";
  if (iequals(ext, ".tiff"))
    return "image/tiff";
  if (iequals(ext, ".tif"))
    return "image/tiff";
  if (iequals(ext, ".svg"))
    return "image/svg+xml";
  if (iequals(ext, ".svgz"))
    return "image/svg+xml";
  return "application/text";
}

http::response<http::string_body> Util::BadRequest(
    const http::request<http::string_body>& req,
    string_view why) {
  http::response<http::string_body> res{http::status::bad_request,
                                        req.version()};
  res.set(http::field::server, BOOST_BEAST_VERSION_STRING);
  res.set(http::field::content_type, "text/html");
  res.keep_alive(req.keep_alive());
  res.body() = why.to_string();
  res.prepare_payload();
  return res;
}

http::response<http::string_body> Util::NotFound(
    const http::request<http::string_body>& req,
    string_view target) {
  http::response<http::string_body> res{http::status::not_found, req.version()};
  res.set(http::field::server, BOOST_BEAST_VERSION_STRING);
  res.set(http::field::content_type, "text/html");
  res.keep_alive(req.keep_alive());
  res.body() = "The resource '" + target.to_string() + "' was not found.";
  res.prepare_payload();
  return res;
}

http::response<http::string_body> Util::ServerError(
    const http::request<http::string_body>& req,
    string_view what) {
  http::response<http::string_body> res{http::status::internal_server_error,
                                        req.version()};
  res.set(http::field::server, BOOST_BEAST_VERSION_STRING);
  res.set(http::field::content_type, "text/html");
  res.keep_alive(req.keep_alive());
  res.body() = "An error occurred: '" + what.to_string() + "'";
  res.prepare_payload();
  return res;
}
