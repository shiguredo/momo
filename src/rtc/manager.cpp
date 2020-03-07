
#include "manager.h"

#include <iostream>

#include "absl/memory/memory.h"
#include "api/audio_codecs/builtin_audio_decoder_factory.h"
#include "api/audio_codecs/builtin_audio_encoder_factory.h"
#include "api/create_peerconnection_factory.h"
#include "api/rtc_event_log/rtc_event_log_factory.h"
#include "api/task_queue/default_task_queue_factory.h"
#include "api/video_track_source_proxy.h"
#include "media/engine/webrtc_media_engine.h"
#include "modules/audio_device/include/audio_device.h"
#include "modules/audio_processing/include/audio_processing.h"
#include "modules/video_capture/video_capture.h"
#include "modules/video_capture/video_capture_factory.h"
#include "observer.h"
#include "rtc_base/logging.h"
#include "rtc_base/openssl_certificate.h"
#include "rtc_base/ssl_adapter.h"
#include "scalable_track_source.h"
#include "util.h"

#ifdef __APPLE__
#include "mac_helper/objc_codec_factory_helper.h"
#else
#include "api/video_codecs/builtin_video_decoder_factory.h"
#include "api/video_codecs/builtin_video_encoder_factory.h"
#endif

#if USE_ROS
#include "ros/ros_audio_device_module.h"
#endif

#if USE_MMAL_ENCODER || USE_JETSON_ENCODER || USE_NVCODEC_ENCODER
#include "api/video_codecs/video_encoder_factory.h"
#include "hw_video_encoder_factory.h"
#endif
#if USE_MMAL_ENCODER || USE_JETSON_ENCODER
#include "api/video_codecs/video_decoder_factory.h"
#include "hw_video_decoder_factory.h"
#endif

#include "ssl_verifier.h"

RTCManager::RTCManager(
    ConnectionSettings conn_settings,
    rtc::scoped_refptr<ScalableVideoTrackSource> video_track_source,
    VideoTrackReceiver* receiver)
    : _conn_settings(conn_settings),
      _receiver(receiver),
      _data_manager(nullptr) {
  rtc::InitializeSSL();

  _networkThread = rtc::Thread::CreateWithSocketServer();
  _networkThread->Start();
  _workerThread = rtc::Thread::Create();
  _workerThread->Start();
  _signalingThread = rtc::Thread::Create();
  _signalingThread->Start();

#if defined(__linux__)
  webrtc::AudioDeviceModule::AudioLayer audio_layer =
      webrtc::AudioDeviceModule::kLinuxAlsaAudio;
#else
  webrtc::AudioDeviceModule::AudioLayer audio_layer =
      webrtc::AudioDeviceModule::kPlatformDefaultAudio;
#endif
  if (_conn_settings.no_audio) {
    audio_layer = webrtc::AudioDeviceModule::kDummyAudio;
  }

  webrtc::PeerConnectionFactoryDependencies dependencies;
  dependencies.network_thread = _networkThread.get();
  dependencies.worker_thread = _workerThread.get();
  dependencies.signaling_thread = _signalingThread.get();
  dependencies.task_queue_factory = webrtc::CreateDefaultTaskQueueFactory();
  dependencies.call_factory = webrtc::CreateCallFactory();
  dependencies.event_log_factory =
      absl::make_unique<webrtc::RtcEventLogFactory>(
          dependencies.task_queue_factory.get());

  // media_dependencies
  cricket::MediaEngineDependencies media_dependencies;
  media_dependencies.task_queue_factory = dependencies.task_queue_factory.get();
#if USE_ROS
  media_dependencies.adm = ROSAudioDeviceModule::Create(
      _conn_settings, dependencies.task_queue_factory.get());
#else
  media_dependencies.adm = webrtc::AudioDeviceModule::Create(
      audio_layer, dependencies.task_queue_factory.get());
#endif
  media_dependencies.audio_encoder_factory =
      webrtc::CreateBuiltinAudioEncoderFactory();
  media_dependencies.audio_decoder_factory =
      webrtc::CreateBuiltinAudioDecoderFactory();
#ifdef __APPLE__
  media_dependencies.video_encoder_factory = CreateObjCEncoderFactory();
  media_dependencies.video_decoder_factory = CreateObjCDecoderFactory();
#else
#if USE_MMAL_ENCODER || USE_JETSON_ENCODER || USE_NVCODEC_ENCODER
  media_dependencies.video_encoder_factory =
      std::unique_ptr<webrtc::VideoEncoderFactory>(
          absl::make_unique<HWVideoEncoderFactory>());
#else
  media_dependencies.video_encoder_factory =
      webrtc::CreateBuiltinVideoEncoderFactory();
#endif
#if USE_MMAL_ENCODER || USE_JETSON_ENCODER
  media_dependencies.video_decoder_factory =
      std::unique_ptr<webrtc::VideoDecoderFactory>(
          absl::make_unique<HWVideoDecoderFactory>());
#else
  media_dependencies.video_decoder_factory =
      webrtc::CreateBuiltinVideoDecoderFactory();
#endif
#endif
  media_dependencies.audio_mixer = nullptr;
  media_dependencies.audio_processing =
      webrtc::AudioProcessingBuilder().Create();

  dependencies.media_engine =
      cricket::CreateMediaEngine(std::move(media_dependencies));

  _factory =
      webrtc::CreateModularPeerConnectionFactory(std::move(dependencies));
  if (!_factory.get()) {
    RTC_LOG(LS_ERROR) << __FUNCTION__
                      << ": Failed to initialize PeerConnectionFactory";
    exit(1);
  }

  webrtc::PeerConnectionFactoryInterface::Options factory_options;
  factory_options.disable_sctp_data_channels = false;
  factory_options.disable_encryption = false;
  factory_options.ssl_max_version = rtc::SSL_PROTOCOL_DTLS_12;
  _factory->SetOptions(factory_options);

  if (!_conn_settings.no_audio) {
    cricket::AudioOptions ao;
    if (_conn_settings.disable_echo_cancellation)
      ao.echo_cancellation = false;
    if (_conn_settings.disable_auto_gain_control)
      ao.auto_gain_control = false;
    if (_conn_settings.disable_noise_suppression)
      ao.noise_suppression = false;
    if (_conn_settings.disable_highpass_filter)
      ao.highpass_filter = false;
    if (_conn_settings.disable_typing_detection)
      ao.typing_detection = false;
    if (_conn_settings.disable_residual_echo_detector)
      ao.residual_echo_detector = false;
    RTC_LOG(LS_INFO) << __FUNCTION__ << ": " << ao.ToString();
    _audio_track = _factory->CreateAudioTrack(Util::generateRandomChars(),
                                              _factory->CreateAudioSource(ao));
    if (!_audio_track) {
      RTC_LOG(LS_WARNING) << __FUNCTION__ << ": Cannot create audio_track";
    }
  }

  if (video_track_source && !_conn_settings.no_video) {
    rtc::scoped_refptr<webrtc::VideoTrackSourceInterface> video_source =
        webrtc::VideoTrackSourceProxy::Create(
            _signalingThread.get(), _workerThread.get(), video_track_source);
    _video_track =
        _factory->CreateVideoTrack(Util::generateRandomChars(), video_source);
    if (_video_track) {
      if (_conn_settings.fixed_resolution) {
        _video_track->set_content_hint(
            webrtc::VideoTrackInterface::ContentHint::kText);
      }
      if (_receiver != nullptr && _conn_settings.show_me) {
        _receiver->AddTrack(_video_track);
      }
    } else {
      RTC_LOG(LS_WARNING) << __FUNCTION__ << ": Cannot create video_track";
    }
  }
}

RTCManager::~RTCManager() {
  _audio_track = nullptr;
  _video_track = nullptr;
  _factory = nullptr;
  _networkThread->Stop();
  _workerThread->Stop();
  _signalingThread->Stop();

  rtc::CleanupSSL();
}

void RTCManager::SetDataManager(RTCDataManager* data_manager) {
  _data_manager = data_manager;
}

class RTCSSLVerifier : public rtc::SSLCertificateVerifier {
 public:
  RTCSSLVerifier() {}
  bool Verify(const rtc::SSLCertificate& certificate) override {
    return SSLVerifier::VerifyX509(
        static_cast<const rtc::OpenSSLCertificate&>(certificate).x509());
  }
};

std::shared_ptr<RTCConnection> RTCManager::createConnection(
    webrtc::PeerConnectionInterface::RTCConfiguration rtc_config,
    RTCMessageSender* sender) {
  rtc_config.enable_dtls_srtp = true;
  rtc_config.sdp_semantics = webrtc::SdpSemantics::kUnifiedPlan;
  std::unique_ptr<PeerConnectionObserver> observer(
      new PeerConnectionObserver(sender, _receiver, _data_manager));
  webrtc::PeerConnectionDependencies dependencies(observer.get());

  // WebRTC の SSL 接続の検証は自前のルート証明書(rtc_base/ssl_roots.h)でやっていて、
  // その中に Let's Encrypt の証明書が無いため、接続先によっては接続できないことがある。
  //
  // それを解消するために tls_cert_verifier を設定して自前で検証を行う。
  dependencies.tls_cert_verifier =
      std::unique_ptr<rtc::SSLCertificateVerifier>(new RTCSSLVerifier());

  rtc::scoped_refptr<webrtc::PeerConnectionInterface> connection =
      _factory->CreatePeerConnection(rtc_config, std::move(dependencies));
  if (!connection) {
    RTC_LOG(LS_ERROR) << __FUNCTION__ << ": CreatePeerConnection failed";
    return nullptr;
  }

  std::string stream_id = Util::generateRandomChars();

  if (_audio_track) {
    webrtc::RTCErrorOr<rtc::scoped_refptr<webrtc::RtpSenderInterface> >
        audio_sender = connection->AddTrack(_audio_track, {stream_id});
    if (!audio_sender.ok()) {
      RTC_LOG(LS_WARNING) << __FUNCTION__ << ": Cannot add _audio_track";
    }
  }

  if (_video_track) {
    webrtc::RTCErrorOr<rtc::scoped_refptr<webrtc::RtpSenderInterface> >
        video_add_result = connection->AddTrack(_video_track, {stream_id});
    if (video_add_result.ok()) {
      rtc::scoped_refptr<webrtc::RtpSenderInterface> video_sender =
          video_add_result.value();
      webrtc::RtpParameters parameters = video_sender->GetParameters();
      parameters.degradation_preference = _conn_settings.getPriority();
      video_sender->SetParameters(parameters);
    } else {
      RTC_LOG(LS_WARNING) << __FUNCTION__ << ": Cannot add _video_track";
    }
  }

  return std::make_shared<RTCConnection>(sender, std::move(observer),
                                         connection);
}
