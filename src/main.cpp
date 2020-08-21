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

#if USE_ROS
#include "ros/ros_log_sink.h"
#include "ros/ros_video_capturer.h"
#else

#if USE_SCREEN_CAPTURER
#include "rtc/screen_video_capturer.h"
#endif

#if defined(__APPLE__)
#include "mac_helper/mac_capturer.h"
#elif defined(__linux__)
#if USE_MMAL_ENCODER
#include "hwenc_mmal/mmal_v4l2_capturer.h"
#endif
#include "v4l2_video_capturer/v4l2_video_capturer.h"
#else
#include "rtc/device_video_capturer.h"
#endif
#endif

#include "serial_data_channel/serial_data_manager.h"

#if USE_SDL2
#include "sdl_renderer/sdl_renderer.h"
#endif

#include "ayame/ayame_client.h"
#include "p2p/p2p_server.h"
#include "rtc/rtc_manager.h"
#include "sora/sora_client.h"
#include "sora/sora_server.h"
#include "util.h"

const size_t kDefaultMaxLogFileSize = 10 * 1024 * 1024;

int main(int argc, char* argv[]) {
  MomoArgs args;

  bool use_test = false;
  bool use_ayame = false;
  bool use_sora = false;
  int log_level = rtc::LS_NONE;

  Util::ParseArgs(argc, argv, use_test, use_ayame, use_sora, log_level, args);

  rtc::LogMessage::LogToDebug((rtc::LoggingSeverity)log_level);
  rtc::LogMessage::LogTimestamps();
  rtc::LogMessage::LogThreads();

#if USE_ROS
  std::unique_ptr<rtc::LogSink> log_sink(new ROSLogSink());
  rtc::LogMessage::AddLogToStream(log_sink.get(), rtc::LS_INFO);
#else
  std::unique_ptr<rtc::FileRotatingLogSink> log_sink(
      new rtc::FileRotatingLogSink("./", "webrtc_logs", kDefaultMaxLogFileSize,
                                   10));
  if (!log_sink->Init()) {
    RTC_LOG(LS_ERROR) << __FUNCTION__ << "Failed to open log file";
    log_sink.reset();
    return 1;
  }
  rtc::LogMessage::AddLogToStream(log_sink.get(), rtc::LS_INFO);
#endif

  auto capturer = ([&]() -> rtc::scoped_refptr<ScalableVideoTrackSource> {
    if (args.no_video_device) {
      return nullptr;
    }

#if USE_SCREEN_CAPTURER
    if (args.screen_capture) {
      RTC_LOG(LS_INFO) << "Screen capturer source list: "
                       << ScreenVideoCapturer::GetSourceListString();
      webrtc::DesktopCapturer::SourceList sources;
      if (!ScreenVideoCapturer::GetSourceList(&sources)) {
        RTC_LOG(LS_ERROR) << __FUNCTION__ << "Failed select screen source";
        return nullptr;
      }
      auto size = args.GetSize();
      rtc::scoped_refptr<ScreenVideoCapturer> capturer(
          new rtc::RefCountedObject<ScreenVideoCapturer>(
              sources[0].id, size.width, size.height, args.framerate));
      return capturer;
    }
#endif

#if USE_ROS
    ROSVideoCapturerConfig ros_vc_config;
    ros_vc_config.camera_name = args.camera_name;
    ros_vc_config.image_compressed = args.image_compressed;
    rtc::scoped_refptr<ROSVideoCapturer> capturer(
        new rtc::RefCountedObject<ROSVideoCapturer>(std::move(ros_vc_config)));
    return capturer;
#else  // USE_ROS
    auto size = args.GetSize();
#if defined(__APPLE__)
    return MacCapturer::Create(size.width, size.height, args.framerate,
                               args.video_device);
#elif defined(__linux__)
    V4L2VideoCapturerConfig v4l2_config;
    v4l2_config.video_device = args.video_device;
    v4l2_config.width = size.width;
    v4l2_config.height = size.height;
    v4l2_config.framerate = args.framerate;
    v4l2_config.force_i420 = args.force_i420;
    v4l2_config.use_native = args.use_native;

#if USE_MMAL_ENCODER
    if (v4l2_config.use_native) {
      MMALV4L2CapturerConfig mmal_config = v4l2_config;
      return MMALV4L2Capturer::Create(std::move(mmal_config));
    } else {
      return V4L2VideoCapturer::Create(std::move(v4l2_config));
    }
#else
    return V4L2VideoCapturer::Create(std::move(v4l2_config));
#endif
#else
    return DeviceVideoCapturer::Create(size.width, size.height, args.framerate,
                                       args.video_device);
#endif
#endif  // USE_ROS
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
  rtcm_config.show_me = args.show_me;
  rtcm_config.simulcast = args.sora_simulcast;

  rtcm_config.disable_echo_cancellation = args.disable_echo_cancellation;
  rtcm_config.disable_auto_gain_control = args.disable_auto_gain_control;
  rtcm_config.disable_noise_suppression = args.disable_noise_suppression;
  rtcm_config.disable_highpass_filter = args.disable_highpass_filter;
  rtcm_config.disable_typing_detection = args.disable_typing_detection;
  rtcm_config.disable_residual_echo_detector =
      args.disable_residual_echo_detector;

  rtcm_config.vp8_encoder = args.vp8_encoder;
  rtcm_config.vp8_decoder = args.vp8_decoder;
  rtcm_config.vp9_encoder = args.vp9_encoder;
  rtcm_config.vp9_decoder = args.vp9_decoder;
  rtcm_config.av1_encoder = args.av1_encoder;
  rtcm_config.av1_decoder = args.av1_decoder;
  rtcm_config.h264_encoder = args.h264_encoder;
  rtcm_config.h264_decoder = args.h264_decoder;

  rtcm_config.priority = args.priority;
#if USE_SDL2
  std::unique_ptr<SDLRenderer> sdl_renderer = nullptr;
  if (args.use_sdl) {
    sdl_renderer.reset(new SDLRenderer(args.window_width, args.window_height,
                                       args.fullscreen));
  }

  std::unique_ptr<RTCManager> rtc_manager(new RTCManager(
      std::move(rtcm_config), std::move(capturer), sdl_renderer.get()));
#else
  std::unique_ptr<RTCManager> rtc_manager(
      new RTCManager(std::move(rtcm_config), std::move(capturer), nullptr));
#endif

  {
    boost::asio::io_context ioc{1};
    boost::asio::executor_work_guard<boost::asio::io_context::executor_type>
        work_guard(ioc.get_executor());

    std::unique_ptr<RTCDataManager> data_manager = nullptr;
    if (!args.serial_device.empty()) {
      data_manager =
          SerialDataManager::Create(ioc, args.serial_device, args.serial_rate);
      if (!data_manager) {
        return 1;
      }
      rtc_manager->SetDataManager(data_manager.get());
    }

    boost::asio::signal_set signals(ioc, SIGINT, SIGTERM);
    signals.async_wait(
        [&](const boost::system::error_code&, int) { ioc.stop(); });

    std::shared_ptr<SoraClient> sora_client;
    std::shared_ptr<AyameClient> ayame_client;

    if (use_sora) {
      SoraClientConfig config;
      config.insecure = args.insecure;
      config.signaling_host = args.sora_signaling_host;
      config.channel_id = args.sora_channel_id;
      config.video = args.sora_video;
      config.audio = args.sora_audio;
      config.video_codec = args.sora_video_codec;
      config.audio_codec = args.sora_audio_codec;
      config.video_bitrate = args.sora_video_bitrate;
      config.audio_bitrate = args.sora_audio_bitrate;
      config.metadata = args.sora_metadata;
      config.role = args.sora_role;
      config.multistream = args.sora_multistream;
      config.spotlight = args.sora_spotlight;
      config.port = args.sora_port;
      config.simulcast = args.sora_simulcast;

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
    }

    if (use_test) {
      P2PServerConfig config;
      config.no_google_stun = args.no_google_stun;
      config.doc_root = args.test_document_root;

      const boost::asio::ip::tcp::endpoint endpoint{
          boost::asio::ip::make_address("0.0.0.0"),
          static_cast<unsigned short>(args.test_port)};
      P2PServer::Create(ioc, endpoint, rtc_manager.get(), std::move(config))
          ->Run();
    }

    if (use_ayame) {
      AyameClientConfig config;
      config.insecure = args.insecure;
      config.no_google_stun = args.no_google_stun;
      config.signaling_host = args.ayame_signaling_host;
      config.room_id = args.ayame_room_id;
      config.client_id = args.ayame_client_id;
      config.signaling_key = args.ayame_signaling_key;

      ayame_client =
          AyameClient::Create(ioc, rtc_manager.get(), std::move(config));
      ayame_client->Connect();
    }

#if USE_SDL2
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
#else
    ioc.run();
#endif
  }

  //この順番は綺麗に落ちるけど、あまり安全ではない
#if USE_SDL2
  sdl_renderer = nullptr;
#endif
  rtc_manager = nullptr;

  return 0;
}
