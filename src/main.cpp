#ifndef _MSC_VER
#include <unistd.h>
#endif
#include <thread>
#include <condition_variable>
#include <iostream>
#include <atomic>
#include <vector>
#include <string>
#include <csignal>
#include <boost/filesystem.hpp>
#include <boost/preprocessor/stringize.hpp>
#include <CLI/CLI.hpp>

#include "rtc_base/logsinks.h"

#include "CivetServer.h"

#include "connection_settings.h"
#include "rtc/manager.h"
#include "ws_client/client.h"
#include "sora/sora_server.h"
#include "p2p/p2p_server.h"

// バージョン情報
#define MOMO_VERSION "18.10.0-rc0"

// HWA を効かせる場合は 1 になる
#if USE_IL_ENCODER
  #define MOMO_USE_IL_ENCODER 1
#else
  #define MOMO_USE_IL_ENCODER 0
#endif

const size_t kDefaultMaxLogFileSize = 10 * 1024 * 1024;

struct signal_waiter {
  void set_running(bool running) {
    std::unique_lock<std::mutex> lock(_mutex);
    _running = running;
    _condition.notify_all();
  }

  void wait_until_running() {
    std::unique_lock<std::mutex> lock(_mutex);
    while (_running) {
      _condition.wait(lock);
    }
  }

private:
  std::mutex _mutex;
  bool _running;
  std::condition_variable _condition;

} _signal_waiter;

static void sigintHandler(int sig)
{
  _signal_waiter.set_running(false);
}


// 列挙した文字列のみを許可するバリデータ
struct Enum : public CLI::Validator {
  Enum(std::vector<std::string> xs) {
    std::stringstream out;

    bool first = true;
    for (auto x: xs) {
      if (!first) {
        out << ",";
      }
      first = false;

      out << x;
    }
    std::string name = out.str();

    tname = "STR in [" + name + "]";
    func = [xs, name](std::string input) {
      auto it = std::find(std::begin(xs), std::end(xs), input);
      if (it == std::end(xs)) {
        return "Value " + input + " not in range [" + name + "]";
      }
      return std::string();
    };
  }
};


int main(int argc, char* argv[])
{
  CLI::App app("Momo - WebRTC ネイティブクライアント");
  ConnectionSettings cs;

  bool is_daemon = false;
  bool version = false;
  int log_level = rtc::LS_NONE;

  _signal_waiter.set_running(true);
  signal(SIGINT, sigintHandler);

  app.add_flag("--no-video", cs.no_video, "ビデオを表示しない");
  app.add_flag("--no-audio", cs.no_audio, "オーディオを出さない");
  app.add_option("--video-codec", cs.video_codec, "ビデオコーデック")->check(Enum({"VP8", "VP9", "H264"}));
  app.add_option("--audio-codec", cs.audio_codec, "オーディオコーデック")->check(Enum({"OPUS", "PCMU"}));
  app.add_option("--video-bitrate", cs.video_bitrate, "ビデオのビットレート")->check(CLI::Range(1, 30000));
  app.add_option("--audio-bitrate", cs.video_bitrate, "オーディオのビットレート")->check(CLI::Range(6, 510));
  app.add_option("--resolution", cs.resolution, "解像度")->check(Enum({"QVGA", "VGA", "HD", "FHD", "4K"}));
  app.add_option("--framerate", cs.framerate, "フレームレート")->check(CLI::Range(1, 60));
  app.add_option("--priority", cs.priority, "優先設定 (Experimental)")->check(Enum({"BALANCE", "FRAMERATE", "RESOLUTION"}));
  app.add_flag("--daemon", is_daemon, "デーモン化する");
  app.add_flag("--version", version, "バージョン情報の表示");
  app.add_option("--log-level", log_level, "ログレベル")->check(CLI::Range(0, 5));
  // 隠しオプション
  app.add_option("--metadata", cs.metadata, "メタデータ")->group("");

  auto p2p_app = app.add_subcommand("p2p", "P2P");
  auto sora_app = app.add_subcommand("sora", "WebRTC SFU Sora");

  p2p_app->add_option("--port", cs.p2p_port, "ポート番号")->check(CLI::Range(0, 65535));

  sora_app->add_option("SIGNALING-URL", cs.sora_signaling_host, "シグナリングホスト")->required();
  sora_app->add_option("CHANNEL-ID", cs.sora_channel_id, "チャンネルID")->required();
  sora_app->add_flag("--auto", cs.sora_auto_connect, "自動接続する");

  try {
    app.parse(argc, argv);
  } catch (const CLI::ParseError &e) {
    return app.exit(e);
  }

  if (version) {
    std::cout << "WebRTC Native Client Momo version " MOMO_VERSION " USE_IL_ENCODER=" BOOST_PP_STRINGIZE(MOMO_USE_IL_ENCODER) << std::endl;
    return 0;
  }

  if (!p2p_app->parsed() && !sora_app->parsed()) {
    std::cout << app.help() << std::endl;
    return 1;
  }

#ifndef _MSC_VER
  if (is_daemon)
  {
    if (daemon(1, 0) == -1)
    {
      std::cerr << "failed to launch momo daemon" << std::endl;
      return -1;
    }
  }
#endif

  std::unique_ptr<rtc::FileRotatingLogSink> log_sink(
      new rtc::FileRotatingLogSink("./", "webrtc_logs", kDefaultMaxLogFileSize, 10));
  if (!log_sink->Init())
  {
    RTC_LOG(LS_ERROR) << __FUNCTION__ << "Failed to open log file";
    log_sink.reset();
    return 1;
  }

  rtc::LogMessage::LogToDebug((rtc::LoggingSeverity)log_level);
  rtc::LogMessage::LogTimestamps();
  rtc::LogMessage::LogThreads();
  rtc::LogMessage::AddLogToStream(log_sink.get(), rtc::LS_INFO);

  std::unique_ptr<RTCManager> rtc_manager(new RTCManager(cs));
  std::unique_ptr<WebSocketClient> ws_client(new WebSocketClient());

  std::string currentPath = boost::filesystem::path(boost::filesystem::current_path()).string();
  std::string port_str = std::to_string(cs.p2p_port);
  const char* options[] = {
    "document_root", currentPath.c_str(),
    "listening_ports", port_str.c_str(),
    0
  };
  std::unique_ptr<CivetServer> server(new CivetServer(options));

  std::unique_ptr<SoraServer> sora_server;
  if (sora_app->parsed()) {
    sora_server.reset(new SoraServer(
          server.get(), rtc_manager.get(), ws_client.get(), cs));
  }

  std::unique_ptr<P2PServer> p2p_server;
  if (p2p_app->parsed()) {
    p2p_server.reset(new P2PServer(
            server.get(), rtc_manager.get(), cs));
  }

  _signal_waiter.wait_until_running();

  p2p_server = nullptr;
  sora_server = nullptr;
  server = nullptr;
  ws_client = nullptr;
  rtc_manager = nullptr;

  return 0;
}
