#include "rtc_manager.h"

#include <iostream>

// WebRTC
#include <absl/memory/memory.h>
#include <api/audio_codecs/builtin_audio_decoder_factory.h>
#include <api/audio_codecs/builtin_audio_encoder_factory.h>
#include <api/create_peerconnection_factory.h>
#include <api/rtc_event_log/rtc_event_log_factory.h>
#include <api/task_queue/default_task_queue_factory.h>
#include <media/engine/webrtc_media_engine.h>
#include <modules/audio_device/include/audio_device.h>
#include <modules/audio_device/include/audio_device_factory.h>
#include <modules/audio_processing/include/audio_processing.h>
#include <modules/video_capture/video_capture.h>
#include <modules/video_capture/video_capture_factory.h>
#include <p2p/client/basic_port_allocator.h>
#include <pc/connection_context.h>
#include <pc/video_track_source_proxy.h>
#include <rtc_base/logging.h>
#include <rtc_base/ssl_adapter.h>

#include "momo_video_decoder_factory.h"
#include "momo_video_encoder_factory.h"
#include "peer_connection_observer.h"
#include "rtc_ssl_verifier.h"
#include "scalable_track_source.h"
#include "util.h"

RTCManager::RTCManager(
    RTCManagerConfig config,
    rtc::scoped_refptr<ScalableVideoTrackSource> video_track_source,
    VideoTrackReceiver* receiver)
    : config_(std::move(config)), receiver_(receiver) {
  rtc::InitializeSSL();

  network_thread_ = rtc::Thread::CreateWithSocketServer();
  network_thread_->Start();
  worker_thread_ = rtc::Thread::Create();
  worker_thread_->Start();
  signaling_thread_ = rtc::Thread::Create();
  signaling_thread_->Start();

#if defined(__linux__)

#if USE_LINUX_PULSE_AUDIO
  webrtc::AudioDeviceModule::AudioLayer audio_layer =
      webrtc::AudioDeviceModule::kLinuxPulseAudio;
#else
  webrtc::AudioDeviceModule::AudioLayer audio_layer =
      webrtc::AudioDeviceModule::kLinuxAlsaAudio;
#endif

#else
  webrtc::AudioDeviceModule::AudioLayer audio_layer =
      webrtc::AudioDeviceModule::kPlatformDefaultAudio;
#endif
  if (config_.no_audio_device) {
    audio_layer = webrtc::AudioDeviceModule::kDummyAudio;
  }

  webrtc::PeerConnectionFactoryDependencies dependencies;
  dependencies.network_thread = network_thread_.get();
  dependencies.worker_thread = worker_thread_.get();
  dependencies.signaling_thread = signaling_thread_.get();
  dependencies.task_queue_factory = webrtc::CreateDefaultTaskQueueFactory();
  dependencies.call_factory = webrtc::CreateCallFactory();
  dependencies.event_log_factory =
      absl::make_unique<webrtc::RtcEventLogFactory>(
          dependencies.task_queue_factory.get());

  // media_dependencies
  cricket::MediaEngineDependencies media_dependencies;
  media_dependencies.task_queue_factory = dependencies.task_queue_factory.get();
#if defined(_WIN32)
  media_dependencies.adm =
      worker_thread_->Invoke<rtc::scoped_refptr<webrtc::AudioDeviceModule>>(
          RTC_FROM_HERE, [&] {
            return webrtc::CreateWindowsCoreAudioAudioDeviceModule(
                dependencies.task_queue_factory.get());
          });
#else
  media_dependencies.adm =
      worker_thread_->Invoke<rtc::scoped_refptr<webrtc::AudioDeviceModule>>(
          RTC_FROM_HERE, [&] {
            return webrtc::AudioDeviceModule::Create(
                audio_layer, dependencies.task_queue_factory.get());
          });
#endif
  media_dependencies.audio_encoder_factory =
      webrtc::CreateBuiltinAudioEncoderFactory();
  media_dependencies.audio_decoder_factory =
      webrtc::CreateBuiltinAudioDecoderFactory();

  {
    auto info = VideoCodecInfo::Get();
    // 名前を短くする
    auto& cf = config_;
    auto resolve = &VideoCodecInfo::Resolve;
    MomoVideoEncoderFactoryConfig ec;
    ec.vp8_encoder = resolve(cf.vp8_encoder, info.vp8_encoders);
    ec.vp9_encoder = resolve(cf.vp9_encoder, info.vp9_encoders);
    ec.av1_encoder = resolve(cf.av1_encoder, info.av1_encoders);
    ec.h264_encoder = resolve(cf.h264_encoder, info.h264_encoders);
    ec.simulcast = cf.simulcast;
    ec.hardware_encoder_only = cf.hardware_encoder_only;
#if defined(__linux__) && USE_NVCODEC_ENCODER
    ec.cuda_context = cf.cuda_context;
#endif
    media_dependencies.video_encoder_factory =
        std::unique_ptr<webrtc::VideoEncoderFactory>(
            absl::make_unique<MomoVideoEncoderFactory>(ec));
    MomoVideoDecoderFactoryConfig dc;
    dc.vp8_decoder = resolve(cf.vp8_decoder, info.vp8_decoders);
    dc.vp9_decoder = resolve(cf.vp9_decoder, info.vp9_decoders);
    dc.av1_decoder = resolve(cf.av1_decoder, info.av1_decoders);
    dc.h264_decoder = resolve(cf.h264_decoder, info.h264_decoders);
#if USE_NVCODEC_ENCODER
    dc.cuda_context = cf.cuda_context;
#endif
    media_dependencies.video_decoder_factory =
        std::unique_ptr<webrtc::VideoDecoderFactory>(
            absl::make_unique<MomoVideoDecoderFactory>(dc));
  }

  media_dependencies.audio_mixer = nullptr;
  media_dependencies.audio_processing =
      webrtc::AudioProcessingBuilder().Create();

  dependencies.media_engine =
      cricket::CreateMediaEngine(std::move(media_dependencies));

  //factory_ =
  //    webrtc::CreateModularPeerConnectionFactory(std::move(dependencies));
  factory_ = dependencies.signaling_thread
                 ->Invoke<rtc::scoped_refptr<CustomPeerConnectionFactory>>(
                     RTC_FROM_HERE, [&dependencies]() {
                       return CustomPeerConnectionFactory::Create(
                           std::move(dependencies));
                     });
  if (!factory_.get()) {
    RTC_LOG(LS_ERROR) << __FUNCTION__
                      << ": Failed to initialize PeerConnectionFactory";
    exit(1);
  }

  webrtc::PeerConnectionFactoryInterface::Options factory_options;
  factory_options.disable_encryption = false;
  factory_options.ssl_max_version = rtc::SSL_PROTOCOL_DTLS_12;
  factory_options.crypto_options.srtp.enable_gcm_crypto_suites = true;
  factory_->SetOptions(factory_options);

  if (!config_.no_audio_device) {
    cricket::AudioOptions ao;
    if (config_.disable_echo_cancellation)
      ao.echo_cancellation = false;
    if (config_.disable_auto_gain_control)
      ao.auto_gain_control = false;
    if (config_.disable_noise_suppression)
      ao.noise_suppression = false;
    if (config_.disable_highpass_filter)
      ao.highpass_filter = false;
    if (config_.disable_residual_echo_detector)
      ao.residual_echo_detector = false;
    RTC_LOG(LS_INFO) << __FUNCTION__ << ": " << ao.ToString();
    audio_track_ = factory_->CreateAudioTrack(Util::GenerateRandomChars(),
                                              factory_->CreateAudioSource(ao));
    if (!audio_track_) {
      RTC_LOG(LS_WARNING) << __FUNCTION__ << ": Cannot create audio_track";
    }
  }

  if (video_track_source && !config_.no_video_device) {
    rtc::scoped_refptr<webrtc::VideoTrackSourceInterface> video_source =
        webrtc::VideoTrackSourceProxy::Create(
            signaling_thread_.get(), worker_thread_.get(), video_track_source);
    video_track_ =
        factory_->CreateVideoTrack(Util::GenerateRandomChars(), video_source);
    if (video_track_) {
      if (config_.fixed_resolution) {
        video_track_->set_content_hint(
            webrtc::VideoTrackInterface::ContentHint::kText);
      }
      if (receiver_ != nullptr && config_.show_me) {
        receiver_->AddTrack(video_track_);
      }
    } else {
      RTC_LOG(LS_WARNING) << __FUNCTION__ << ": Cannot create video_track";
    }
  }
}

RTCManager::~RTCManager() {
  audio_track_ = nullptr;
  video_track_ = nullptr;
  factory_ = nullptr;
  network_thread_->Stop();
  worker_thread_->Stop();
  signaling_thread_->Stop();

  rtc::CleanupSSL();
}

void RTCManager::AddDataManager(std::shared_ptr<RTCDataManager> data_manager) {
  data_manager_dispatcher_.Add(data_manager);
}

class RawCryptString : public rtc::CryptStringImpl {
 public:
  RawCryptString(const std::string& str) : str_(str) {}
  size_t GetLength() const override { return str_.size(); }
  void CopyTo(char* dest, bool nullterminate) const override {
    for (int i = 0; i < str_.size(); i++) {
      *dest++ = str_[i];
    }
    if (nullterminate) {
      *dest = '\0';
    }
  }
  std::string UrlEncode() const override { throw std::exception(); }
  CryptStringImpl* Copy() const { return new RawCryptString(str_); }
  void CopyRawTo(std::vector<unsigned char>* dest) const {
    dest->assign(str_.begin(), str_.end());
  }

 private:
  std::string str_;
};

std::shared_ptr<RTCConnection> RTCManager::CreateConnection(
    webrtc::PeerConnectionInterface::RTCConfiguration rtc_config,
    RTCMessageSender* sender) {
  rtc_config.sdp_semantics = webrtc::SdpSemantics::kUnifiedPlan;
  std::unique_ptr<PeerConnectionObserver> observer(
      new PeerConnectionObserver(sender, receiver_, &data_manager_dispatcher_));
  webrtc::PeerConnectionDependencies dependencies(observer.get());

  // WebRTC の SSL 接続の検証は自前のルート証明書(rtc_base/ssl_roots.h)でやっていて、
  // その中に Let's Encrypt の証明書が無いため、接続先によっては接続できないことがある。
  //
  // それを解消するために tls_cert_verifier を設定して自前で検証を行う。
  dependencies.tls_cert_verifier = std::unique_ptr<rtc::SSLCertificateVerifier>(
      new RTCSSLVerifier(config_.insecure));

  auto context = factory_->GetContext();
  dependencies.allocator.reset(new cricket::BasicPortAllocator(
      context->default_network_manager(), context->default_socket_factory(),
      rtc_config.turn_customizer));
  dependencies.allocator->SetPortRange(
      rtc_config.port_allocator_config.min_port,
      rtc_config.port_allocator_config.max_port);
  dependencies.allocator->set_flags(rtc_config.port_allocator_config.flags);
  if (config_.proxy_type != rtc::PROXY_NONE) {
    RTC_LOG(LS_INFO) << "Set Proxy: type="
                     << rtc::ProxyToString(config_.proxy_type)
                     << " url=" << config_.proxy_hostname << ":"
                     << config_.proxy_port
                     << " username=" << config_.proxy_username;
    rtc::ProxyInfo pi;
    pi.type = config_.proxy_type;
    if (!config_.proxy_hostname.empty() && config_.proxy_port >= 0) {
      pi.address =
          rtc::SocketAddress(config_.proxy_hostname, config_.proxy_port);
    }
    if (!config_.proxy_username.empty()) {
      pi.username = config_.proxy_username;
    }
    if (!config_.proxy_password.empty()) {
      pi.password = rtc::CryptString(RawCryptString(config_.proxy_password));
    }
    dependencies.allocator->set_proxy(config_.proxy_agent, pi);
  }

  webrtc::RTCErrorOr<rtc::scoped_refptr<webrtc::PeerConnectionInterface>>
      connection = factory_->CreatePeerConnectionOrError(
          rtc_config, std::move(dependencies));
  if (!connection.ok()) {
    RTC_LOG(LS_ERROR) << __FUNCTION__ << ": CreatePeerConnection failed";
    return nullptr;
  }

  return std::make_shared<RTCConnection>(sender, std::move(observer),
                                         connection.value());
}

void RTCManager::InitTracks(RTCConnection* conn) {
  auto connection = conn->GetConnection();

  std::string stream_id = Util::GenerateRandomChars();

  if (audio_track_) {
    webrtc::RTCErrorOr<rtc::scoped_refptr<webrtc::RtpSenderInterface>>
        audio_sender = connection->AddTrack(audio_track_, {stream_id});
    if (!audio_sender.ok()) {
      RTC_LOG(LS_WARNING) << __FUNCTION__ << ": Cannot add audio_track_";
    }
  }

  if (video_track_) {
    webrtc::RTCErrorOr<rtc::scoped_refptr<webrtc::RtpSenderInterface>>
        video_add_result = connection->AddTrack(video_track_, {stream_id});
    if (video_add_result.ok()) {
      video_sender_ = video_add_result.value();
    } else {
      RTC_LOG(LS_WARNING) << __FUNCTION__ << ": Cannot add video_track_";
    }
  }
}

void RTCManager::SetParameters() {
  if (!video_sender_) {
    return;
  }

  webrtc::RtpParameters parameters = video_sender_->GetParameters();
  parameters.degradation_preference = config_.GetPriority();
  video_sender_->SetParameters(parameters);
}
