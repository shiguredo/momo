#include "util.h"

#include "rtc_base/helpers.h"
#include <CLI/CLI.hpp>
#include <nlohmann/json.hpp>
#include <boost/beast/version.hpp>
#include <boost/preprocessor/stringize.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>
#if USE_ROS
#include "ros/ros.h"
#endif

// バージョン情報
// 通常は外から渡すが、渡されていなかった場合の対応
#ifndef MOMO_VERSION
#define MOMO_VERSION "internal-build"
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

void Util::parseArgs(int argc, char *argv[], bool &is_daemon,
                        bool &use_p2p, bool &use_sora, bool &use_ayame,
                        int &log_level, ConnectionSettings &cs)
{
  ros::init(argc, argv, "momo", ros::init_options::AnonymousName);

  ros::NodeHandle nh;
  cs.camera_name = nh.resolveName("image");
  cs.audio_topic_name = nh.resolveName("audio");

  ros::NodeHandle local_nh("~");
  local_nh.param<bool>("compressed", cs.image_compressed, cs.image_compressed);

  local_nh.param<bool>("use_p2p", use_p2p, use_p2p);
  local_nh.param<bool>("use_sora", use_sora, use_sora);
  local_nh.param<bool>("use_ayame", use_ayame, use_ayame);

  local_nh.param<bool>("no_video", cs.no_video, cs.no_video);
  local_nh.param<bool>("no_audio", cs.no_audio, cs.no_audio);
  local_nh.param<std::string>("video_codec", cs.video_codec, cs.video_codec);
  local_nh.param<std::string>("audio_codec", cs.audio_codec, cs.audio_codec);
  local_nh.param<int>("video_bitrate", cs.video_bitrate, cs.video_bitrate);
  local_nh.param<int>("audio_bitrate", cs.audio_bitrate, cs.audio_bitrate);
  local_nh.param<std::string>("resolution", cs.resolution, cs.resolution);
  local_nh.param<int>("framerate", cs.framerate, cs.framerate);
  local_nh.param<int>("audio_topic_rate", cs.audio_topic_rate, cs.audio_topic_rate);
  local_nh.param<int>("audio_topic_ch", cs.audio_topic_ch, cs.audio_topic_ch);
  local_nh.param<std::string>("priority", cs.priority, cs.priority);
  local_nh.param<int>("port", cs.port, cs.port);
  local_nh.param<int>("log_level", log_level, log_level);

  if (use_sora && local_nh.hasParam("SIGNALING_URL") && local_nh.hasParam("CHANNEL_ID")) {
    local_nh.getParam("SIGNALING_URL", cs.sora_signaling_host);
    local_nh.getParam("CHANNEL_ID", cs.sora_channel_id);
    local_nh.param<bool>("auto", cs.sora_auto_connect, cs.sora_auto_connect);

    // 隠しオプション
    std::string sora_metadata;
    local_nh.param<std::string>("metadata", sora_metadata, "");

    // メタデータのパース
    if (!sora_metadata.empty())
    {
      cs.sora_metadata = json::parse(sora_metadata);
    }
  } else if (use_p2p) {
    local_nh.param<std::string>("document_root", cs.p2p_document_root, get_current_dir_name());
  } else if (use_ayame && local_nh.hasParam("SIGNALING_URL") && local_nh.hasParam("CLIENT_ID") && && local_nh.hasParam("ROOM_ID")) {
    local_nh.getParam("SIGNALING_URL", cs.ayame_signaling_host);
    local_nh.getParam("CLIENT_ID", cs.ayame_client_id);
    local_nh.getParam("ROOM_ID", cs.ayame_room_id);
  } else {
    exit(1);
  }
}

#else

void Util::parseArgs(int argc, char *argv[], bool &is_daemon,
                        bool &use_p2p, bool &use_sora, 
                        int &log_level, ConnectionSettings &cs)
{
  CLI::App app("Momo - WebRTC ネイティブクライアント");

  bool version = false;

  app.add_flag("--no-video", cs.no_video, "ビデオを表示しない");
  app.add_flag("--no-audio", cs.no_audio, "オーディオを出さない");
  app.add_set("--resolution", cs.resolution, {"QVGA", "VGA", "HD", "FHD", "4K"},
              "解像度");
  app.add_option("--framerate", cs.framerate, "フレームレート")->check(CLI::Range(1, 60));
  app.add_flag("--fixed-resolution", cs.fixed_resolution, "固定解像度");
  app.add_set("--priority", cs.priority, {"BALANCE", "FRAMERATE", "RESOLUTION"},
              "優先設定 (Experimental)");
  app.add_option("--port", cs.port, "ポート番号(デフォルト:8080)")
      ->check(CLI::Range(0, 65535));
  app.add_flag("--daemon", is_daemon, "デーモン化する");
  app.add_flag("--version", version, "バージョン情報の表示");
  auto log_level_map = std::vector<std::pair<std::string, int>>(
      {{"verbose", 0}, {"info", 1}, {"warning", 2}, {"error", 3}, {"none", 4}});
  app.add_option("--log-level", log_level, "ログレベル")
      ->transform(CLI::CheckedTransformer(log_level_map, CLI::ignore_case));

  auto p2p_app = app.add_subcommand("p2p", "P2P");
  auto sora_app = app.add_subcommand("sora", "WebRTC SFU Sora");
  auto ayame_app = app.add_subcommand("ayame", "WebRTC Signaling Server Ayame");

  p2p_app
      ->add_option("--document-root", cs.p2p_document_root, "配信ディレクトリ")
      ->check(CLI::ExistingDirectory);

  sora_app->add_option("SIGNALING-URL", cs.sora_signaling_host, "シグナリングホスト")->required();
  sora_app->add_option("CHANNEL-ID", cs.sora_channel_id, "チャンネルID")->required();
  sora_app->add_flag("--auto", cs.sora_auto_connect, "自動接続する");
#if MOMO_USE_H264
  sora_app->add_set("--video-codec", cs.video_codec, {"VP8", "VP9", "H264"},
                    "ビデオコーデック");
#else
  sora_app->add_set("--video-codec", cs.video_codec, {"VP8", "VP9"},
                    "ビデオコーデック");
#endif
  sora_app->add_set("--audio-codec", cs.audio_codec, {"OPUS", "PCMU"},
                    "オーディオコーデック");
  sora_app
      ->add_option("--video-bitrate", cs.video_bitrate, "ビデオのビットレート")
      ->check(CLI::Range(1, 30000));
  sora_app
      ->add_option("--audio-bitrate", cs.audio_bitrate,
                   "オーディオのビットレート")
      ->check(CLI::Range(6, 510));

  // 隠しオプション
  auto is_json = CLI::Validator(
      [](std::string input) -> std::string {
        try {
          auto _ = json::parse(input);
          return std::string();
        } catch (json::parse_error &e) {
          return "Value " + input + " is not JSON Value";
        }
      },
      "JSON Value");
  std::string sora_metadata;
  sora_app->add_option("--metadata", sora_metadata, "メタデータ")
      ->group("")
      ->check(is_json);

  ayame_app->add_option("SIGNALING-URL", cs.ayame_signaling_host, "シグナリングホスト")->required();
  ayame_app->add_option("CLIENT-ID", cs.ayame_client_id, "クライアントID")->required();
  ayame_app->add_option("ROOM-ID", cs.ayame_room_id, "ルームID")->required();

  ayame_app->add_option("SIGNALING-URL", cs.ayame_signaling_host, "シグナリングホスト")->required();
  ayame_app->add_option("CLIENT-ID", cs.ayame_client_id, "クライアントID")->required();
  ayame_app->add_option("ROOM-ID", cs.ayame_room_id, "ルームID")->required();

  try
  {
    app.parse(argc, argv);
  }
  catch (const CLI::ParseError &e)
  {
    exit(app.exit(e));
  }

  // メタデータのパース
  if (!sora_metadata.empty()) {
    cs.sora_metadata = json::parse(sora_metadata);
  }

  if (cs.p2p_document_root.empty()) {
    cs.p2p_document_root = boost::filesystem::current_path().string();
  }

  if (version)
  {
    std::cout << "WebRTC Native Client Momo version " MOMO_VERSION " USE_MMAL_ENCODER=" BOOST_PP_STRINGIZE(MOMO_USE_MMAL_ENCODER) << std::endl;
    exit(0);
  }

  if (!p2p_app->parsed() && !sora_app->parsed() && !ayame_app->parsed())
  {
    std::cout << app.help() << std::endl;
    exit(1);
  }

  if (sora_app->parsed()) {
    use_sora = true;
  }

  if (p2p_app->parsed()) {
    use_p2p = true;
  }

  if (ayame_app->parsed()) {
    use_ayame = true;
  }
}

#endif

std::string Util::generateRundomChars()
{
  return generateRundomChars(32);
}

std::string Util::generateRundomChars(size_t length)
{
  std::string result;
  rtc::CreateRandomString(length, &result);
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
    auto const ext = [&path]
    {
        auto const pos = path.rfind(".");
        if(pos == string_view::npos)
            return string_view{};
        return path.substr(pos);
    }();

    if (iequals(ext, ".htm"))  return "text/html";
    if (iequals(ext, ".html")) return "text/html";
    if (iequals(ext, ".php"))  return "text/html";
    if (iequals(ext, ".css"))  return "text/css";
    if (iequals(ext, ".txt"))  return "text/plain";
    if (iequals(ext, ".js"))   return "application/javascript";
    if (iequals(ext, ".json")) return "application/json";
    if (iequals(ext, ".xml"))  return "application/xml";
    if (iequals(ext, ".swf"))  return "application/x-shockwave-flash";
    if (iequals(ext, ".flv"))  return "video/x-flv";
    if (iequals(ext, ".png"))  return "image/png";
    if (iequals(ext, ".jpe"))  return "image/jpeg";
    if (iequals(ext, ".jpeg")) return "image/jpeg";
    if (iequals(ext, ".jpg"))  return "image/jpeg";
    if (iequals(ext, ".gif"))  return "image/gif";
    if (iequals(ext, ".bmp"))  return "image/bmp";
    if (iequals(ext, ".ico"))  return "image/vnd.microsoft.icon";
    if (iequals(ext, ".tiff")) return "image/tiff";
    if (iequals(ext, ".tif"))  return "image/tiff";
    if (iequals(ext, ".svg"))  return "image/svg+xml";
    if (iequals(ext, ".svgz")) return "image/svg+xml";
    return "application/text";
}

http::response<http::string_body> Util::badRequest(const http::request<http::string_body>& req, string_view why) {
    http::response<http::string_body> res{http::status::bad_request, req.version()};
    res.set(http::field::server, BOOST_BEAST_VERSION_STRING);
    res.set(http::field::content_type, "text/html");
    res.keep_alive(req.keep_alive());
    res.body() = why.to_string();
    res.prepare_payload();
    return res;
}

http::response<http::string_body> Util::notFound(const http::request<http::string_body>& req, string_view target) {
    http::response<http::string_body> res{http::status::not_found, req.version()};
    res.set(http::field::server, BOOST_BEAST_VERSION_STRING);
    res.set(http::field::content_type, "text/html");
    res.keep_alive(req.keep_alive());
    res.body() = "The resource '" + target.to_string() + "' was not found.";
    res.prepare_payload();
    return res;
}

http::response<http::string_body> Util::serverError(const http::request<http::string_body>& req, string_view what) {
    http::response<http::string_body> res{http::status::internal_server_error, req.version()};
    res.set(http::field::server, BOOST_BEAST_VERSION_STRING);
    res.set(http::field::content_type, "text/html");
    res.keep_alive(req.keep_alive());
    res.body() = "An error occurred: '" + what.to_string() + "'";
    res.prepare_payload();
    return res;
}
