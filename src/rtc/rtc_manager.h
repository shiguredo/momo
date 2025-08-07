#ifndef RTC_MANAGER_H_
#define RTC_MANAGER_H_

#include <memory>

// WebRTC
#include <api/environment/environment_factory.h>
#include <api/peer_connection_interface.h>
#include <pc/connection_context.h>
#include <pc/peer_connection_factory.h>
#include <pc/video_track_source.h>

#include "rtc_connection.h"
#include "rtc_data_manager_dispatcher.h"
#include "rtc_message_sender.h"
#include "sora/cuda_context.h"
#include "sora/scalable_track_source.h"
#include "video_codec_info.h"
#include "video_track_receiver.h"

#if defined(USE_FAKE_CAPTURE_DEVICE)
// Forward declaration
class FakeAudioCapturer;
#endif

// webrtc::PeerConnectionFactory から ConnectionContext を取り出す方法が無いので、
// 継承して無理やり使えるようにする
class CustomPeerConnectionFactory : public webrtc::PeerConnectionFactory {
 public:
  CustomPeerConnectionFactory(
      webrtc::PeerConnectionFactoryDependencies dependencies)
      : CustomPeerConnectionFactory(
            webrtc::ConnectionContext::Create(webrtc::CreateEnvironment(),
                                              &dependencies),
            &dependencies) {}
  CustomPeerConnectionFactory(
      webrtc::scoped_refptr<webrtc::ConnectionContext> context,
      webrtc::PeerConnectionFactoryDependencies* dependencies)
      : conn_context_(context),
        webrtc::PeerConnectionFactory(context, dependencies) {}

  static webrtc::scoped_refptr<CustomPeerConnectionFactory> Create(
      webrtc::PeerConnectionFactoryDependencies dependencies) {
    return webrtc::make_ref_counted<CustomPeerConnectionFactory>(
        std::move(dependencies));
  }

  webrtc::scoped_refptr<webrtc::ConnectionContext> GetContext() const {
    return conn_context_;
  }

 private:
  webrtc::scoped_refptr<webrtc::ConnectionContext> conn_context_;
};

struct RTCManagerConfig {
  bool insecure = false;

  bool no_video_device = false;
  bool no_audio_device = false;

  bool fixed_resolution = false;
  bool simulcast = false;
  bool hardware_encoder_only = false;

  bool disable_echo_cancellation = false;
  bool disable_auto_gain_control = false;
  bool disable_noise_suppression = false;
  bool disable_highpass_filter = false;

  VideoCodecInfo::Type vp8_encoder = VideoCodecInfo::Type::Default;
  VideoCodecInfo::Type vp8_decoder = VideoCodecInfo::Type::Default;
  VideoCodecInfo::Type vp9_encoder = VideoCodecInfo::Type::Default;
  VideoCodecInfo::Type vp9_decoder = VideoCodecInfo::Type::Default;
  VideoCodecInfo::Type av1_encoder = VideoCodecInfo::Type::Default;
  VideoCodecInfo::Type av1_decoder = VideoCodecInfo::Type::Default;
  VideoCodecInfo::Type h264_encoder = VideoCodecInfo::Type::Default;
  VideoCodecInfo::Type h264_decoder = VideoCodecInfo::Type::Default;
  VideoCodecInfo::Type h265_encoder = VideoCodecInfo::Type::Default;
  VideoCodecInfo::Type h265_decoder = VideoCodecInfo::Type::Default;

  std::string openh264;

  std::string priority = "FRAMERATE";

  webrtc::DegradationPreference GetPriority() {
    if (priority == "FRAMERATE") {
      return webrtc::DegradationPreference::MAINTAIN_FRAMERATE;
    } else if (priority == "RESOLUTION") {
      return webrtc::DegradationPreference::MAINTAIN_RESOLUTION;
    }
    return webrtc::DegradationPreference::BALANCED;
  }

#if defined(USE_NVCODEC_ENCODER)
  std::shared_ptr<sora::CudaContext> cuda_context;
#endif

  std::string proxy_url;
  std::string proxy_username;
  std::string proxy_password;
  
#if defined(USE_FAKE_CAPTURE_DEVICE)
  webrtc::scoped_refptr<FakeAudioCapturer> fake_audio_capturer;
#endif
};

class RTCManager {
 public:
  RTCManager(
      RTCManagerConfig config,
      webrtc::scoped_refptr<sora::ScalableVideoTrackSource> video_track_source,
      VideoTrackReceiver* receiver);
  ~RTCManager();
  void AddDataManager(std::shared_ptr<RTCDataManager> data_manager);
  std::shared_ptr<RTCConnection> CreateConnection(
      webrtc::PeerConnectionInterface::RTCConfiguration rtc_config,
      RTCMessageSender* sender);
  void InitTracks(RTCConnection* conn);
  void SetParameters();

 private:
  webrtc::scoped_refptr<webrtc::PeerConnectionFactoryInterface> factory_;
  webrtc::scoped_refptr<webrtc::ConnectionContext> context_;
  webrtc::scoped_refptr<webrtc::AudioTrackInterface> audio_track_;
  webrtc::scoped_refptr<webrtc::VideoTrackInterface> video_track_;
  webrtc::scoped_refptr<webrtc::RtpSenderInterface> video_sender_;
  std::unique_ptr<webrtc::Thread> network_thread_;
  std::unique_ptr<webrtc::Thread> worker_thread_;
  std::unique_ptr<webrtc::Thread> signaling_thread_;
  RTCManagerConfig config_;
  VideoTrackReceiver* receiver_;
  RTCDataManagerDispatcher data_manager_dispatcher_;
};

#endif
