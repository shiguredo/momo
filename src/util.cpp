#include "util.h"

#include "rtc_base/helpers.h"
#include <CLI/CLI.hpp>
#include <nlohmann/json.hpp>
#include <boost/preprocessor/stringize.hpp>
#include <boost/filesystem.hpp>
#if USE_ROS
#include "ros/ros.h"
#endif

// バージョン情報
// 通常は外から渡すが、渡されていなかった場合の対応
#ifndef MOMO_VERSION
#define MOMO_VERSION "internal-build"
#endif

// HWA を効かせる場合は 1 になる
#if USE_IL_ENCODER
  #define MOMO_USE_IL_ENCODER 1
#else
  #define MOMO_USE_IL_ENCODER 0
#endif

// H264 を有効にする場合は 1 になる
#if USE_H264
  #define MOMO_USE_H264 1
#else
  #define MOMO_USE_H264 0
#endif

using json = nlohmann::json;

// 列挙した文字列のみを許可するバリデータ
struct Enum : public CLI::Validator
{
  Enum(std::vector<std::string> xs)
  {
    std::stringstream out;

    bool first = true;
    for (auto x : xs)
    {
      if (!first)
      {
        out << ",";
      }
      first = false;

      out << x;
    }
    std::string name = out.str();

    tname = "STR in [" + name + "]";
    func = [xs, name](std::string input) {
      auto it = std::find(std::begin(xs), std::end(xs), input);
      if (it == std::end(xs))
      {
        return "Value " + input + " not in range [" + name + "]";
      }
      return std::string();
    };
  }
};

// JSON Value のみを許可するバリデータ
struct JsonValue : public CLI::Validator
{
  JsonValue()
  {
    tname = "JSON Value";
    func = [](std::string input) {
      try
      {
        json::parse(input);
        return std::string();
      }
      catch (json::parse_error &e)
      {
        return "Value " + input + " is not JSON Value";
      }
    };
  }
};

// ディレクトリが存在するか確認するバリデータ
struct DirectoryExists : public CLI::Validator {
  DirectoryExists() {
    tname = "Directory";
    func = [](std::string input) {
      auto path = boost::filesystem::path(input);
      auto st = boost::filesystem::status(path);
      if (!boost::filesystem::exists(st)) {
        return "Path " + input + " not exists";
      }
      if (!boost::filesystem::is_directory(st)) {
        return "Path " + input + " is not directory";
      }

      return std::string();
    };
  }
};

#if USE_ROS

void Util::parseArgs(int argc, char *argv[], bool &is_daemon,
                        bool &use_p2p, bool &use_sora, 
                        int &log_level, ConnectionSettings &cs)
{
  ros::init(argc, argv, "momo", ros::init_options::AnonymousName);

  ros::NodeHandle nh;
  cs.camera_name = nh.resolveName("image");

  ros::NodeHandle local_nh("~");
  local_nh.param<bool>("compressed", cs.image_compressed, cs.image_compressed);

  local_nh.param<bool>("use_p2p", use_p2p, use_p2p);
  local_nh.param<bool>("use_sora", use_sora, use_sora);

  local_nh.param<bool>("no_video", cs.no_video, cs.no_video);
  local_nh.param<bool>("no_audio", cs.no_audio, cs.no_audio);
  local_nh.param<std::string>("video_codec", cs.video_codec, cs.video_codec);
  local_nh.param<std::string>("audio_codec", cs.audio_codec, cs.audio_codec);
  local_nh.param<int>("video_bitrate", cs.video_bitrate, cs.video_bitrate);
  local_nh.param<int>("audio_bitrate", cs.audio_bitrate, cs.audio_bitrate);
  local_nh.param<std::string>("resolution", cs.resolution, cs.resolution);
  local_nh.param<int>("framerate", cs.framerate, cs.framerate);
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
#if MOMO_USE_H264
  app.add_option("--video-codec", cs.video_codec, "ビデオコーデック")->check(Enum({"VP8", "VP9", "H264"}));
#else
  app.add_option("--video-codec", cs.video_codec, "ビデオコーデック")->check(Enum({"VP8", "VP9"}));
#endif
  app.add_option("--audio-codec", cs.audio_codec, "オーディオコーデック")->check(Enum({"OPUS", "PCMU"}));
  app.add_option("--video-bitrate", cs.video_bitrate, "ビデオのビットレート")->check(CLI::Range(1, 30000));
  app.add_option("--audio-bitrate", cs.audio_bitrate, "オーディオのビットレート")->check(CLI::Range(6, 510));
  app.add_option("--resolution", cs.resolution, "解像度")->check(Enum({"QVGA", "VGA", "HD", "FHD", "4K"}));
  app.add_option("--framerate", cs.framerate, "フレームレート")->check(CLI::Range(1, 60));
  app.add_flag("--fixed-resolution", cs.fixed_resolution, "固定解像度");
  app.add_option("--priority", cs.priority, "優先設定 (Experimental)")->check(Enum({"BALANCE", "FRAMERATE", "RESOLUTION"}));
  app.add_option("--port", cs.port, "ポート番号")->check(CLI::Range(0, 65535));
  app.add_flag("--daemon", is_daemon, "デーモン化する");
  app.add_flag("--version", version, "バージョン情報の表示");
  app.add_option("--log-level", log_level, "ログレベル")->check(CLI::Range(0, 5));

  auto p2p_app = app.add_subcommand("p2p", "P2P");
  auto sora_app = app.add_subcommand("sora", "WebRTC SFU Sora");

  p2p_app->add_option("--document-root", cs.p2p_document_root, "配信ディレクトリ")->check(DirectoryExists());

  sora_app->add_option("SIGNALING-URL", cs.sora_signaling_host, "シグナリングホスト")->required();
  sora_app->add_option("CHANNEL-ID", cs.sora_channel_id, "チャンネルID")->required();
  sora_app->add_flag("--auto", cs.sora_auto_connect, "自動接続する");

  // 隠しオプション
  std::string sora_metadata;
  sora_app->add_option("--metadata", sora_metadata, "メタデータ")->group("")->check(JsonValue());

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
    std::cout << "WebRTC Native Client Momo version " MOMO_VERSION " USE_IL_ENCODER=" BOOST_PP_STRINGIZE(MOMO_USE_IL_ENCODER) << std::endl;
    exit(0);
  }

  if (!p2p_app->parsed() && !sora_app->parsed())
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
