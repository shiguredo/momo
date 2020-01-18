#include "util.h"

#include <regex>

// external libraries
#include <CLI/CLI.hpp>
#include <boost/beast/version.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/preprocessor/stringize.hpp>
#include <nlohmann/json.hpp>

#include "momo_version.h"
#include "rtc_base/helpers.h"
#if USE_ROS
#include "ros/ros.h"
#endif

// HWA を効かせる場合は 1 になる
#if USE_MMAL_ENCODER
#define MOMO_USE_MMAL_ENCODER 1
#else
#define MOMO_USE_MMAL_ENCODER 0
#endif

// H264 を有効にする場合は 1 になる
#if USE_H264
#define MOMO_USE_H264 1
#else
#define MOMO_USE_H264 0
#endif

using json = nlohmann::json;

#if USE_ROS

void Util::parseArgs(int argc,
                     char* argv[],
                     bool& is_daemon,
                     bool& use_test,
                     bool& use_ayame,
                     bool& use_sora,
                     int& log_level,
                     ConnectionSettings& cs) {
  ros::init(argc, argv, "momo", ros::init_options::AnonymousName);

  ros::NodeHandle nh;
  cs.camera_name = nh.resolveName("image");
  cs.audio_topic_name = nh.resolveName("audio");

  ros::NodeHandle local_nh("~");
  local_nh.param<bool>("compressed", cs.image_compressed, cs.image_compressed);

  local_nh.param<bool>("use_test", use_test, use_test);
  local_nh.param<bool>("use_ayame", use_ayame, use_ayame);
  local_nh.param<bool>("use_sora", use_sora, use_sora);

  local_nh.param<bool>("no_video", cs.no_video, cs.no_video);
  local_nh.param<bool>("no_audio", cs.no_audio, cs.no_audio);
  local_nh.param<bool>("force_i420", cs.force_i420, cs.force_i420);
  local_nh.param<bool>("use_native", cs.use_native, cs.use_native);
#if USE_MMAL_ENCODER || USE_JETSON_ENCODER
  local_nh.param<std::string>("video_device", cs.video_device, cs.video_device);
#endif
  local_nh.param<std::string>("video_codec", cs.video_codec, cs.video_codec);
  local_nh.param<std::string>("audio_codec", cs.audio_codec, cs.audio_codec);
  local_nh.param<int>("video_bitrate", cs.video_bitrate, cs.video_bitrate);
  local_nh.param<int>("audio_bitrate", cs.audio_bitrate, cs.audio_bitrate);
  local_nh.param<std::string>("resolution", cs.resolution, cs.resolution);
  local_nh.param<int>("framerate", cs.framerate, cs.framerate);
  local_nh.param<int>("audio_topic_rate", cs.audio_topic_rate,
                      cs.audio_topic_rate);
  local_nh.param<int>("audio_topic_ch", cs.audio_topic_ch, cs.audio_topic_ch);
  local_nh.param<std::string>("priority", cs.priority, cs.priority);
  local_nh.param<int>("port", cs.port, cs.port);
  local_nh.param<int>("log_level", log_level, log_level);

  // オーディオフラグ
  local_nh.param<bool>("disable_echo_cancellation",
                       cs.disable_echo_cancellation,
                       cs.disable_echo_cancellation);
  local_nh.param<bool>("disable_auto_gain_control",
                       cs.disable_auto_gain_control,
                       cs.disable_auto_gain_control);
  local_nh.param<bool>("disable_noise_suppression",
                       cs.disable_noise_suppression,
                       cs.disable_noise_suppression);
  local_nh.param<bool>("disable_highpass_filter", cs.disable_highpass_filter,
                       cs.disable_highpass_filter);
  local_nh.param<bool>("disable_typing_detection", cs.disable_typing_detection,
                       cs.disable_typing_detection);
  local_nh.param<bool>("disable_residual_echo_detector",
                       cs.disable_residual_echo_detector,
                       cs.disable_residual_echo_detector);

  if (use_sora && local_nh.hasParam("SIGNALING_URL") &&
      local_nh.hasParam("CHANNEL_ID")) {
    local_nh.getParam("SIGNALING_URL", cs.sora_signaling_host);
    local_nh.getParam("CHANNEL_ID", cs.sora_channel_id);
    local_nh.param<bool>("auto", cs.sora_auto_connect, cs.sora_auto_connect);

    std::string sora_metadata;
    local_nh.param<std::string>("metadata", sora_metadata, "");

    // メタデータのパース
    if (!sora_metadata.empty()) {
      cs.sora_metadata = json::parse(sora_metadata);
    }
  } else if (use_test) {
    local_nh.param<std::string>("document_root", cs.test_document_root,
                                get_current_dir_name());
  } else if (use_ayame && local_nh.hasParam("SIGNALING_URL") &&
             local_nh.hasParam("ROOM_ID")) {
    local_nh.getParam("SIGNALING_URL", cs.ayame_signaling_host);
    local_nh.getParam("ROOM_ID", cs.ayame_room_id);
    local_nh.param<std::string>("client_id", cs.ayame_client_id,
                                cs.ayame_client_id);
    local_nh.param<std::string>("signaling_key", cs.ayame_signaling_key,
                                cs.ayame_signaling_key);
  } else {
    exit(1);
  }
}

#else

void Util::parseArgs(int argc,
                     char* argv[],
                     bool& is_daemon,
                     bool& use_test,
                     bool& use_ayame,
                     bool& use_sora,
                     int& log_level,
                     ConnectionSettings& cs) {
  CLI::App app("Momo - WebRTC Native Client");
  app.set_help_all_flag("--help-all",
                        "Print help message for all modes and exit");

  bool version = false;

  auto is_valid_force_i420 = CLI::Validator(
      [](std::string input) -> std::string {
#if USE_MMAL_ENCODER || USE_JETSON_ENCODER
        return std::string();
#else
        return "Not available because your device does not have this feature.";
#endif
      },
      "");
  auto is_valid_use_native = CLI::Validator(
      [](std::string input) -> std::string {
#if USE_MMAL_ENCODER || USE_JETSON_ENCODER
        return std::string();
#else
        return "Not available because your device does not have this feature.";
#endif
      },
      "");

  auto is_valid_h264 = CLI::Validator(
      [](std::string input) -> std::string {
#if MOMO_USE_H264
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
#if USE_SDL
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

  app.add_flag("--no-video", cs.no_video, "Do not send video");
  app.add_flag("--no-audio", cs.no_audio, "Do not send audio");
  app.add_flag(
         "--force-i420", cs.force_i420,
         "Prefer I420 format for video capture (only on supported devices)")
      ->check(is_valid_force_i420);
  app.add_flag("--use-native", cs.use_native,
               "Perform MJPEG deoode and video resize by hardware acceleration "
               "(only on supported devices)")
      ->check(is_valid_use_native);
#if defined(__APPLE__)
  app.add_option("--video-device", cs.video_device,
                 "Use the video device specified by an index or a name "
                 "(use the first one if not specified)");
#elif defined(__linux__)
  app.add_option("--video-device", cs.video_device,
                 "Use the video input device specified by a name "
                 "(some device will be used if not specified)")
      ->check(CLI::ExistingFile);
#endif
  app.add_option("--resolution", cs.resolution,
                 "Video resolution (one of QVGA, VGA, HD, FHD, 4K, or "
                 "[WIDTH]x[HEIGHT])")
      ->check(is_valid_resolution);
  app.add_option("--framerate", cs.framerate, "Video framerate")
      ->check(CLI::Range(1, 60));
  app.add_flag("--fixed-resolution", cs.fixed_resolution,
               "Maintain video resolution in degradation");
  app.add_set("--priority", cs.priority, {"BALANCE", "FRAMERATE", "RESOLUTION"},
              "Preference in video degradation (experimental)");
  app.add_option("--port", cs.port, "Port number (default: 8080)")
      ->check(CLI::Range(0, 65535));
  app.add_flag("--use-sdl", cs.use_sdl,
               "Show video using SDL (if SDL is available)")
      ->check(is_sdl_available);
  app.add_flag("--show-me", cs.show_me, "Show self video (if SDL is available)")
      ->check(is_sdl_available);
  app.add_option("--window-width", cs.window_width,
                 "Window width for videos (if SDL is available)")
      ->check(is_sdl_available)
      ->check(CLI::Range(180, 16384));
  app.add_option("--window-height", cs.window_height,
                 "Window height for videos (if SDL is available)")
      ->check(is_sdl_available)
      ->check(CLI::Range(180, 16384));
  app.add_flag("--fullscreen", cs.fullscreen,
               "Use fullscreen window for videos (if SDL is available)")
      ->check(is_sdl_available);
  app.add_flag("--daemon", is_daemon, "Run as a daemon process");
  app.add_flag("--version", version, "Show version information");
  auto log_level_map = std::vector<std::pair<std::string, int> >(
      {{"verbose", 0}, {"info", 1}, {"warning", 2}, {"error", 3}, {"none", 4}});
  app.add_option("--log-level", log_level, "Log severity level threshold")
      ->transform(CLI::CheckedTransformer(log_level_map, CLI::ignore_case));

  // オーディオフラグ
  app.add_flag("--disable-echo-cancellation", cs.disable_echo_cancellation,
               "Disable echo cancellation for audio");
  app.add_flag("--disable-auto-gain-control", cs.disable_auto_gain_control,
               "Disable auto gain control for audio");
  app.add_flag("--disable-noise-suppression", cs.disable_noise_suppression,
               "Disable noise suppression for audio");
  app.add_flag("--disable-highpass-filter", cs.disable_highpass_filter,
               "Disable highpass filter for audio");
  app.add_flag("--disable-typing-detection", cs.disable_typing_detection,
               "Disable typing detection for audio");
  app.add_flag("--disable-residual-echo-detector",
               cs.disable_residual_echo_detector,
               "Disable residual echo detector for audio");

  auto is_serial_setting_format = CLI::Validator(
      [](std::string input) -> std::string {
        try {
          auto separater_pos = input.find(',');
          std::string bandrate_str = input.substr(separater_pos + 1);
          unsigned int _ = std::stoi(bandrate_str);
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

  auto test_app = app.add_subcommand(
      "test", "Mode for momo development with simple HTTP server");
  auto ayame_app = app.add_subcommand(
      "ayame", "Mode for working with WebRTC Signaling Server Ayame");
  auto sora_app =
      app.add_subcommand("sora", "Mode for working with WebRTC SFU Sora");

  test_app
      ->add_option("--document-root", cs.test_document_root,
                   "HTTP document root directory")
      ->check(CLI::ExistingDirectory);

  ayame_app
      ->add_option("SIGNALING-URL", cs.ayame_signaling_host, "Signaling URL")
      ->required();
  ayame_app->add_option("ROOM-ID", cs.ayame_room_id, "Room ID")->required();
  ayame_app->add_option("--client-id", cs.ayame_client_id, "Client ID");
  ayame_app->add_option("--signaling-key", cs.ayame_signaling_key,
                        "Signaling key");

  sora_app->add_option("SIGNALING-URL", cs.sora_signaling_host, "Signaling URL")
      ->required();
  sora_app->add_option("CHANNEL-ID", cs.sora_channel_id, "Channel ID")
      ->required();
  sora_app->add_flag("--auto", cs.sora_auto_connect,
                     "Connect to Sora automatically");
  sora_app
      ->add_set("--video-codec", cs.video_codec, {"VP8", "VP9", "H264"},
                "Video codec for send")
      ->check(is_valid_h264);
  sora_app->add_set("--audio-codec", cs.audio_codec, {"OPUS", "PCMU"},
                    "Audio codec for send");
  sora_app->add_option("--video-bitrate", cs.video_bitrate, "Video bitrate")
      ->check(CLI::Range(1, 30000));
  sora_app->add_option("--audio-bitrate", cs.audio_bitrate, "Audio bitrate")
      ->check(CLI::Range(6, 510));
  sora_app->add_flag("--multistream", cs.sora_multistream, "Use multistream");
  sora_app->add_set(
      "--role", cs.sora_role,
      {"upstream", "downstream", "sendonly", "recvonly", "sendrecv"},
      "Role (default: upstream)");
  sora_app
      ->add_option("--spotlight", cs.sora_spotlight,
                   "Stream count delivered in spotlight")
      ->check(CLI::Range(1, 10));

  auto is_json = CLI::Validator(
      [](std::string input) -> std::string {
        try {
          auto _ = json::parse(input);
          return std::string();
        } catch (json::parse_error& e) {
          return "Value " + input + " is not JSON Value";
        }
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

  if (!serial_setting.empty()) {
    auto separater_pos = serial_setting.find(',');
    std::string bandrate_str = serial_setting.substr(separater_pos + 1);
    cs.serial_device = serial_setting.substr(0, separater_pos);
    cs.serial_rate = std::stoi(bandrate_str);
  }

  // メタデータのパース
  if (!sora_metadata.empty()) {
    cs.sora_metadata = json::parse(sora_metadata);
  }

  if (cs.test_document_root.empty()) {
    cs.test_document_root = boost::filesystem::current_path().string();
  }

  if (version) {
    std::cout << MomoVersion::GetClientName()
              << " USE_MMAL_ENCODER=" BOOST_PP_STRINGIZE(MOMO_USE_MMAL_ENCODER)
              << std::endl;
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

#endif

std::string Util::generateRandomChars() {
  return generateRandomChars(32);
}

std::string Util::generateRandomChars(size_t length) {
  std::string result;
  rtc::CreateRandomString(length, &result);
  return result;
}

std::string Util::generateRandomNumericChars(size_t length) {
  auto randomNumerics = []() -> char {
    const char charset[] = "0123456789";
    const size_t max_index = (sizeof(charset) - 1);
    return charset[rand() % max_index];
  };
  std::string result(length, 0);
  std::generate_n(result.begin(), length, randomNumerics);
  return result;
}

std::string Util::iceConnectionStateToString(
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

string_view Util::mimeType(string_view path) {
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

http::response<http::string_body> Util::badRequest(
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

http::response<http::string_body> Util::notFound(
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

http::response<http::string_body> Util::serverError(
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
