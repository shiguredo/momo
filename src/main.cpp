#include <atomic>
#include <condition_variable>
#include <csignal>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

// WebRTC
#include <rtc_base/log_sinks.h>
#include <rtc_base/string_utils.h>

#if defined(USE_SCREEN_CAPTURER)
#include "rtc/screen_video_capturer.h"
#endif

#if defined(__APPLE__)
#include "mac_helper/mac_capturer.h"
#elif defined(__linux__)
#if defined(USE_JETSON_ENCODER)
#include "sora/hwenc_jetson/jetson_v4l2_capturer.h"
#elif defined(USE_NVCODEC_ENCODER)
#include "sora/hwenc_nvcodec/nvcodec_v4l2_capturer.h"
#elif defined(USE_V4L2_ENCODER)
#include "hwenc_v4l2/libcamera_capturer.h"
#include "hwenc_v4l2/v4l2_capturer.h"
#endif
#include "sora/v4l2/v4l2_video_capturer.h"
#else
#include "rtc/device_video_capturer.h"
#endif

#include "serial_data_channel/serial_data_manager.h"

#include "sdl_renderer/sdl_renderer.h"

#include "ayame/ayame_client.h"
#include "metrics/metrics_server.h"
#include "p2p/p2p_server.h"
#include "rtc/rtc_manager.h"
#include "sora/sora_client.h"
#include "sora/sora_server.h"
#include "util.h"

#ifdef _WIN32
#include <rtc_base/win/scoped_com_initializer.h>
#endif

#if defined(USE_NVCODEC_ENCODER)
#include "sora/cuda_context.h"
#endif

const size_t kDefaultMaxLogFileSize = 10 * 1024 * 1024;

int main(int argc, char* argv[]) {
#ifdef _WIN32
  webrtc::ScopedCOMInitializer com_initializer(
      webrtc::ScopedCOMInitializer::kMTA);
  if (!com_initializer.Succeeded()) {
    std::cerr << "CoInitializeEx failed" << std::endl;
    return 1;
  }
#endif

  MomoArgs args;

  bool use_test = false;
  bool use_ayame = false;
  bool use_sora = false;
  int log_level = rtc::LS_NONE;

  Util::ParseArgs(argc, argv, use_test, use_ayame, use_sora, log_level, args);

  rtc::LogMessage::LogToDebug((rtc::LoggingSeverity)log_level);
  rtc::LogMessage::LogTimestamps();
  rtc::LogMessage::LogThreads();

  std::unique_ptr<rtc::FileRotatingLogSink> log_sink(
      new rtc::FileRotatingLogSink("./", "webrtc_logs", kDefaultMaxLogFileSize,
                                   10));
  if (!log_sink->Init()) {
    RTC_LOG(LS_ERROR) << __FUNCTION__ << "Failed to open log file";
    log_sink.reset();
    return 1;
  }
  rtc::LogMessage::AddLogToStream(log_sink.get(), rtc::LS_INFO);

#if defined(USE_NVCODEC_ENCODER)
  auto cuda_context = sora::CudaContext::Create();

  // NvCodec が有効な環境で HW MJPEG デコーダを使う場合、CUDA が有効である必要がある
  if (args.hw_mjpeg_decoder && cuda_context == nullptr) {
    std::cerr << "Specified --hw-mjpeg-decoder=true but CUDA is invalid."
              << std::endl;
    return 2;
  }
#endif

  auto capturer = ([&]() -> rtc::scoped_refptr<sora::ScalableVideoTrackSource> {
    if (args.no_video_device) {
      return nullptr;
    }

#if defined(USE_SCREEN_CAPTURER)
    if (args.screen_capture) {
      RTC_LOG(LS_INFO) << "Screen capturer source list: "
                       << ScreenVideoCapturer::GetSourceListString();
      webrtc::DesktopCapturer::SourceList sources;
      if (!ScreenVideoCapturer::GetSourceList(&sources)) {
        RTC_LOG(LS_ERROR) << __FUNCTION__ << "Failed select screen source";
        return nullptr;
      }
      auto size = args.GetSize();
      return rtc::make_ref_counted<ScreenVideoCapturer>(
          sources[0].id, size.width, size.height, args.framerate);
    }
#endif

    auto size = args.GetSize();
#if defined(__APPLE__)
    return MacCapturer::Create(size.width, size.height, args.framerate,
                               args.video_device);
#elif defined(__linux__)
    sora::V4L2VideoCapturerConfig v4l2_config;
    v4l2_config.video_device = args.video_device;
    v4l2_config.width = size.width;
    v4l2_config.height = size.height;
    v4l2_config.framerate = args.framerate;
    v4l2_config.force_i420 = args.force_i420;
    v4l2_config.use_native = args.hw_mjpeg_decoder;

#if defined(USE_JETSON_ENCODER)
    if (v4l2_config.use_native) {
      return sora::JetsonV4L2Capturer::Create(std::move(v4l2_config));
    } else {
      return sora::V4L2VideoCapturer::Create(std::move(v4l2_config));
    }
#elif defined(USE_NVCODEC_ENCODER)
    if (v4l2_config.use_native) {
      sora::NvCodecV4L2CapturerConfig nvcodec_config = v4l2_config;
      nvcodec_config.cuda_context = cuda_context;
      return sora::NvCodecV4L2Capturer::Create(std::move(nvcodec_config));
    } else {
      return sora::V4L2VideoCapturer::Create(std::move(v4l2_config));
    }
#elif defined(USE_V4L2_ENCODER)
    if (args.use_libcamera) {
      LibcameraCapturerConfig libcamera_config = v4l2_config;
      // use_libcamera_native == true でも、サイマルキャストの場合はネイティブフレームを出力しない
      libcamera_config.native_frame_output =
          args.use_libcamera_native && !(use_sora && args.sora_simulcast);
      return LibcameraCapturer::Create(libcamera_config);
    } else if (v4l2_config.use_native && !(use_sora && args.sora_simulcast)) {
      return V4L2Capturer::Create(std::move(v4l2_config));
    } else {
      return sora::V4L2VideoCapturer::Create(std::move(v4l2_config));
    }
#else
    return sora::V4L2VideoCapturer::Create(std::move(v4l2_config));
#endif
#else
    return DeviceVideoCapturer::Create(size.width, size.height, args.framerate,
                                       args.video_device);
#endif
  })();

  if (!capturer && !args.no_video_device) {
    std::cerr << "failed to create capturer" << std::endl;
    return 1;
  }

  RTCManagerConfig rtcm_config;
  rtcm_config.insecure = args.insecure;

  rtcm_config.no_video_device = args.no_video_device;
  rtcm_config.no_audio_device = args.no_audio_device;

  rtcm_config.fixed_resolution = args.fixed_resolution;
  rtcm_config.simulcast = args.sora_simulcast;
  rtcm_config.hardware_encoder_only = args.hw_mjpeg_decoder;

  rtcm_config.disable_echo_cancellation = args.disable_echo_cancellation;
  rtcm_config.disable_auto_gain_control = args.disable_auto_gain_control;
  rtcm_config.disable_noise_suppression = args.disable_noise_suppression;
  rtcm_config.disable_highpass_filter = args.disable_highpass_filter;

  rtcm_config.vp8_encoder = args.vp8_encoder;
  rtcm_config.vp8_decoder = args.vp8_decoder;
  rtcm_config.vp9_encoder = args.vp9_encoder;
  rtcm_config.vp9_decoder = args.vp9_decoder;
  rtcm_config.av1_encoder = args.av1_encoder;
  rtcm_config.av1_decoder = args.av1_decoder;
  rtcm_config.h264_encoder = args.h264_encoder;
  rtcm_config.h264_decoder = args.h264_decoder;
  rtcm_config.h265_encoder = args.h265_encoder;
  rtcm_config.h265_decoder = args.h265_decoder;

  rtcm_config.priority = args.priority;

#if defined(USE_NVCODEC_ENCODER)
  rtcm_config.cuda_context = cuda_context;
#endif

  rtcm_config.proxy_url = args.proxy_url;
  rtcm_config.proxy_username = args.proxy_username;
  rtcm_config.proxy_password = args.proxy_password;

  std::unique_ptr<SDLRenderer> sdl_renderer = nullptr;
  if (args.use_sdl) {
    sdl_renderer.reset(new SDLRenderer(args.window_width, args.window_height,
                                       args.fullscreen));
  }

  std::unique_ptr<RTCManager> rtc_manager(new RTCManager(
      std::move(rtcm_config), std::move(capturer), sdl_renderer.get()));

  {
    boost::asio::io_context ioc{1};
    boost::asio::executor_work_guard<boost::asio::io_context::executor_type>
        work_guard(ioc.get_executor());

    std::shared_ptr<RTCDataManager> data_manager = nullptr;
    if (!args.serial_device.empty()) {
      data_manager = std::shared_ptr<RTCDataManager>(
          SerialDataManager::Create(ioc, args.serial_device, args.serial_rate)
              .release());
      if (!data_manager) {
        return 1;
      }
      rtc_manager->AddDataManager(data_manager);
    }

    boost::asio::signal_set signals(ioc, SIGINT, SIGTERM);
    signals.async_wait(
        [&](const boost::system::error_code&, int) { ioc.stop(); });

    std::shared_ptr<SoraClient> sora_client;
    std::shared_ptr<AyameClient> ayame_client;
    std::shared_ptr<P2PServer> p2p_server;

    MetricsServerConfig metrics_config;
    std::shared_ptr<StatsCollector> stats_collector;

    if (use_sora) {
      SoraClientConfig config;
      config.insecure = args.insecure;
      config.signaling_urls = args.sora_signaling_urls;
      config.channel_id = args.sora_channel_id;
      config.video = args.sora_video;
      config.audio = args.sora_audio;
      config.video_codec_type = args.sora_video_codec_type;
      config.audio_codec_type = args.sora_audio_codec_type;
      config.video_bit_rate = args.sora_video_bit_rate;
      config.audio_bit_rate = args.sora_audio_bit_rate;
      config.metadata = args.sora_metadata;
      config.role = args.sora_role;
      config.spotlight = args.sora_spotlight;
      config.spotlight_number = args.sora_spotlight_number;
      config.port = args.sora_port;
      config.simulcast = args.sora_simulcast;
      config.data_channel_signaling = args.sora_data_channel_signaling;
      config.data_channel_signaling_timeout =
          args.sora_data_channel_signaling_timeout;
      config.ignore_disconnect_websocket =
          args.sora_ignore_disconnect_websocket;
      config.disconnect_wait_timeout = args.sora_disconnect_wait_timeout;
      config.client_cert = args.client_cert;
      config.client_key = args.client_key;
      config.proxy_url = args.proxy_url;
      config.proxy_username = args.proxy_username;
      config.proxy_password = args.proxy_password;

      sora_client =
          SoraClient::Create(ioc, rtc_manager.get(), std::move(config));

      // SoraServer を起動しない場合と、SoraServer を起動して --auto が指定されている場合は即座に接続する。
      // SoraServer を起動するけど --auto が指定されていない場合、SoraServer の API が呼ばれるまで接続しない。
      if (args.sora_port < 0 || args.sora_port >= 0 && args.sora_auto_connect) {
        sora_client->Connect();
      }

      if (args.sora_port >= 0) {
        SoraServerConfig config;
        const boost::asio::ip::tcp::endpoint endpoint{
            boost::asio::ip::make_address("127.0.0.1"),
            static_cast<unsigned short>(args.sora_port)};
        SoraServer::Create(ioc, endpoint, sora_client, rtc_manager.get(),
                           config)
            ->Run();
      }

      stats_collector = sora_client;
    }

    if (use_test) {
      P2PServerConfig config;
      config.no_google_stun = args.no_google_stun;
      config.doc_root = args.test_document_root;

      const boost::asio::ip::tcp::endpoint endpoint{
          boost::asio::ip::make_address("0.0.0.0"),
          static_cast<unsigned short>(args.test_port)};
      p2p_server = P2PServer::Create(ioc, endpoint, rtc_manager.get(),
                                     std::move(config));
      p2p_server->Run();

      stats_collector = p2p_server;
    }

    if (use_ayame) {
      AyameClientConfig config;
      config.insecure = args.insecure;
      config.no_google_stun = args.no_google_stun;
      config.client_cert = args.client_cert;
      config.client_key = args.client_key;
      config.signaling_url = args.ayame_signaling_url;
      config.room_id = args.ayame_room_id;
      config.client_id = args.ayame_client_id;
      config.signaling_key = args.ayame_signaling_key;

      ayame_client =
          AyameClient::Create(ioc, rtc_manager.get(), std::move(config));
      ayame_client->Connect();

      stats_collector = ayame_client;
    }

    if (args.metrics_port >= 0) {
      const boost::asio::ip::tcp::endpoint metrics_endpoint{
          boost::asio::ip::make_address(
              args.metrics_allow_external_ip ? "0.0.0.0" : "127.0.0.1"),
          static_cast<unsigned short>(args.metrics_port)};
      MetricsServer::Create(ioc, metrics_endpoint, rtc_manager.get(),
                            stats_collector, std::move(metrics_config))
          ->Run();
    }

    if (sdl_renderer) {
      sdl_renderer->SetDispatchFunction([&ioc](std::function<void()> f) {
        if (ioc.stopped())
          return;
        boost::asio::dispatch(ioc.get_executor(), f);
      });

      ioc.run();

      sdl_renderer->SetDispatchFunction(nullptr);
    } else {
      ioc.run();
    }
  }

  //この順番は綺麗に落ちるけど、あまり安全ではない
  sdl_renderer = nullptr;

  return 0;
}
