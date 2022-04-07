#ifndef RTC_MANAGER_H_
#define RTC_MANAGER_H_

#include <memory>

// WebRTC
#include <api/peer_connection_interface.h>
#include <pc/video_track_source.h>

#include "cuda/cuda_context.h"
#include "rtc_connection.h"
#include "rtc_data_manager_dispatcher.h"
#include "rtc_message_sender.h"
#include "scalable_track_source.h"
#include "video_codec_info.h"
#include "video_track_receiver.h"

#if USE_MSDK_ENCODER
#include "hwenc_msdk/msdk_session.h"
#endif

struct RTCManagerConfig {
  bool insecure = false;

  bool no_video_device = false;
  bool no_audio_device = false;

  bool fixed_resolution = false;
  bool show_me = false;
  bool simulcast = false;
  bool hardware_encoder_only = false;

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

  std::string priority = "FRAMERATE";

  webrtc::DegradationPreference GetPriority() {
    if (priority == "FRAMERATE") {
      return webrtc::DegradationPreference::MAINTAIN_FRAMERATE;
    } else if (priority == "RESOLUTION") {
      return webrtc::DegradationPreference::MAINTAIN_RESOLUTION;
    }
    return webrtc::DegradationPreference::BALANCED;
  }

#if USE_NVCODEC_ENCODER
  std::shared_ptr<CudaContext> cuda_context;
#endif
#if USE_MSDK_ENCODER
  std::shared_ptr<MsdkSession> msdk_session;
#endif
};

class RTCManager {
 public:
  RTCManager(RTCManagerConfig config,
             rtc::scoped_refptr<ScalableVideoTrackSource> video_track_source,
             VideoTrackReceiver* receiver);
  ~RTCManager();
  void AddDataManager(std::shared_ptr<RTCDataManager> data_manager);
  std::shared_ptr<RTCConnection> CreateConnection(
      webrtc::PeerConnectionInterface::RTCConfiguration rtc_config,
      RTCMessageSender* sender);
  void InitTracks(RTCConnection* conn);
  void SetParameters();

 private:
  rtc::scoped_refptr<webrtc::PeerConnectionFactoryInterface> factory_;
  rtc::scoped_refptr<webrtc::AudioTrackInterface> audio_track_;
  rtc::scoped_refptr<webrtc::VideoTrackInterface> video_track_;
  rtc::scoped_refptr<webrtc::RtpSenderInterface> video_sender_;
  std::unique_ptr<rtc::Thread> network_thread_;
  std::unique_ptr<rtc::Thread> worker_thread_;
  std::unique_ptr<rtc::Thread> signaling_thread_;
  RTCManagerConfig config_;
  VideoTrackReceiver* receiver_;
  RTCDataManagerDispatcher data_manager_dispatcher_;
};

#endif
