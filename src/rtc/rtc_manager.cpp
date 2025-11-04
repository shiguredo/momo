#include "rtc_manager.h"

#include <algorithm>
#include <cctype>
#include <exception>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

// WebRTC
#include <absl/memory/memory.h>
#include <absl/strings/match.h>
#include <absl/strings/numbers.h>
#include <api/audio/builtin_audio_processing_builder.h>
#include <api/audio/create_audio_device_module.h>
#include <api/audio_codecs/builtin_audio_decoder_factory.h>
#include <api/audio_codecs/builtin_audio_encoder_factory.h>
#include <api/create_peerconnection_factory.h>
#include <api/enable_media.h>
#include <api/environment/environment_factory.h>
#include <api/rtc_event_log/rtc_event_log_factory.h>
#include <media/engine/webrtc_media_engine.h>
#include <modules/audio_device/include/audio_device.h>
#include <modules/audio_device/include/audio_device_factory.h>
#include <modules/audio_processing/include/audio_processing.h>
#include <modules/video_capture/video_capture.h>
#include <modules/video_capture/video_capture_factory.h>
#include <p2p/client/basic_port_allocator.h>
#include <pc/peer_connection_factory_proxy.h>
#include <pc/video_track_source_proxy.h>
#include <rtc_base/logging.h>
#include <rtc_base/ssl_adapter.h>

#include "device_info.h"
#include "momo_video_decoder_factory.h"
#include "momo_video_encoder_factory.h"
#include "peer_connection_observer.h"
#include "rtc_ssl_verifier.h"
#include "sora/scalable_track_source.h"
#include "url_parts.h"
#include "util.h"

#if defined(USE_FAKE_CAPTURE_DEVICE)
#include "rtc/fake_audio_capturer.h"
#endif

#if defined(__APPLE__)
namespace {

// デバイス識別子の完全一致判定（大文字小文字を区別しない）
bool MatchDeviceIdentifier(const std::string& target,
                           const char* name,
                           const char* guid) {
  return absl::EqualsIgnoreCase(target, name) ||
         absl::EqualsIgnoreCase(target, guid);
}

// デバイス名/インデックスからインデックスを解決
bool ResolveDeviceIndex(const std::vector<AudioDeviceInfo>& infos,
                        const std::string& device_spec,
                        bool is_input,
                        uint16_t* resolved_index) {
  if (absl::SimpleAtoi(device_spec, resolved_index)) {
    if (*resolved_index >= infos.size()) {
      RTC_LOG(LS_WARNING) << __FUNCTION__
                          << ": Device index out of range. index="
                          << resolved_index << " available=" << infos.size();
      return false;
    }
  } else {
    for (const auto& info : infos) {
      if (MatchDeviceIdentifier(device_spec, info.name.c_str(),
                                info.guid.c_str())) {
        *resolved_index = info.index;
        return true;
      }
    }
  }
  return true;
}

// デバイス名/インデックスからデバイスを設定する
bool SetAudioDevice(webrtc::scoped_refptr<webrtc::AudioDeviceModule> adm,
                    const std::string& device_spec,
                    bool is_input) {
  if (device_spec.empty()) {
    return true;
  }

  auto infos = GetAudioDeviceInfos(adm, is_input);

  uint16_t resolved_index = 0;
  if (!ResolveDeviceIndex(infos, device_spec, is_input, &resolved_index)) {
    return false;
  }

  RTC_LOG(LS_INFO) << __FUNCTION__
                   << ": Applying device index=" << resolved_index
                   << " is_input=" << is_input;

  // デバイスを設定
  int32_t set_result = is_input ? adm->SetRecordingDevice(resolved_index)
                                : adm->SetPlayoutDevice(resolved_index);
  if (set_result != 0) {
    RTC_LOG(LS_WARNING) << __FUNCTION__
                        << ": Failed to set audio device. index="
                        << resolved_index << " is_input=" << is_input
                        << " result=" << set_result;
    return false;
  }

  return true;
}

}  // namespace

#endif

RTCManager::RTCManager(
    RTCManagerConfig config,
    webrtc::scoped_refptr<sora::ScalableVideoTrackSource> video_track_source,
    VideoTrackReceiver* receiver)
    : config_(std::move(config)), receiver_(receiver) {
  webrtc::InitializeSSL();

  network_thread_ = webrtc::Thread::CreateWithSocketServer();
  network_thread_->Start();
  worker_thread_ = webrtc::Thread::Create();
  worker_thread_->Start();
  signaling_thread_ = webrtc::Thread::Create();
  signaling_thread_->Start();

#if defined(__linux__)

#if defined(USE_LINUX_PULSE_AUDIO)
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

  auto env = webrtc::CreateEnvironment();

  webrtc::PeerConnectionFactoryDependencies dependencies;
  dependencies.network_thread = network_thread_.get();
  dependencies.worker_thread = worker_thread_.get();
  dependencies.signaling_thread = signaling_thread_.get();
  dependencies.event_log_factory =
      absl::make_unique<webrtc::RtcEventLogFactory>(&env.task_queue_factory());

  dependencies.adm = worker_thread_->BlockingCall(
      [&]() -> webrtc::scoped_refptr<webrtc::AudioDeviceModule> {
        // create_adm が設定されている場合は、それを使って ADM を作成する
        webrtc::scoped_refptr<webrtc::AudioDeviceModule> adm;
        if (config_.create_adm) {
          return config_.create_adm();
        } else {
#if defined(_WIN32)
          return webrtc::CreateWindowsCoreAudioAudioDeviceModule(
              &env.task_queue_factory());
#else
          return webrtc::CreateAudioDeviceModule(webrtc::CreateEnvironment(),
                                                 audio_layer);
#endif
        }
      });
  dependencies.audio_encoder_factory =
      webrtc::CreateBuiltinAudioEncoderFactory();
  dependencies.audio_decoder_factory =
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
    ec.h265_encoder = resolve(cf.h265_encoder, info.h265_encoders);
    ec.simulcast = cf.simulcast;
    ec.hardware_encoder_only = cf.hardware_encoder_only;
#if defined(USE_NVCODEC_ENCODER)
    ec.cuda_context = cf.cuda_context;
#endif
    ec.openh264 = cf.openh264;
    dependencies.video_encoder_factory =
        std::unique_ptr<webrtc::VideoEncoderFactory>(
            absl::make_unique<MomoVideoEncoderFactory>(ec));
    MomoVideoDecoderFactoryConfig dc;
    dc.vp8_decoder = resolve(cf.vp8_decoder, info.vp8_decoders);
    dc.vp9_decoder = resolve(cf.vp9_decoder, info.vp9_decoders);
    dc.av1_decoder = resolve(cf.av1_decoder, info.av1_decoders);
    dc.h264_decoder = resolve(cf.h264_decoder, info.h264_decoders);
    dc.h265_decoder = resolve(cf.h265_decoder, info.h265_decoders);
#if defined(USE_NVCODEC_ENCODER)
    dc.cuda_context = cf.cuda_context;
#endif
    dependencies.video_decoder_factory =
        std::unique_ptr<webrtc::VideoDecoderFactory>(
            absl::make_unique<MomoVideoDecoderFactory>(dc));
  }

  dependencies.audio_mixer = nullptr;
  dependencies.audio_processing_builder =
      std::make_unique<webrtc::BuiltinAudioProcessingBuilder>();

  webrtc::EnableMedia(dependencies);

  // あとでデバイス選択に使うため保存しておく
  auto adm = dependencies.adm;

  //factory_ =
  //    webrtc::CreateModularPeerConnectionFactory(std::move(dependencies));
  using result_type =
      std::pair<webrtc::scoped_refptr<webrtc::PeerConnectionFactoryInterface>,
                webrtc::scoped_refptr<webrtc::ConnectionContext>>;
  auto p = dependencies.signaling_thread->BlockingCall(
      [&dependencies]() -> result_type {
        auto factory =
            CustomPeerConnectionFactory::Create(std::move(dependencies));
        if (factory == nullptr) {
          return result_type(nullptr, nullptr);
        }
        auto context = factory->GetContext();
        auto proxy = webrtc::PeerConnectionFactoryProxy::Create(
            factory->signaling_thread(), factory->worker_thread(), factory);
        return result_type(proxy, context);
      });
  factory_ = p.first;
  context_ = p.second;
  if (!factory_.get()) {
    RTC_LOG(LS_ERROR) << __FUNCTION__
                      << ": Failed to initialize PeerConnectionFactory";
    exit(1);
  }

  webrtc::PeerConnectionFactoryInterface::Options factory_options;
  factory_options.disable_encryption = false;
  factory_options.ssl_max_version = webrtc::SSL_PROTOCOL_DTLS_12;
  factory_options.crypto_options.srtp.enable_gcm_crypto_suites = true;
  factory_->SetOptions(factory_options);

#if defined(__APPLE__)
  if (adm) {
    worker_thread_->BlockingCall([&]() {
      if (!config_.audio_input_device.empty()) {
        if (!SetAudioDevice(adm, config_.audio_input_device, true)) {
          RTC_LOG(LS_WARNING)
              << __FUNCTION__
              << ": Failed to reapply audio input device. Using default.";
        }
      }
      if (!config_.audio_output_device.empty()) {
        if (!SetAudioDevice(adm, config_.audio_output_device, false)) {
          RTC_LOG(LS_WARNING)
              << __FUNCTION__
              << ": Failed to reapply audio output device. Using default.";
        }
      }
    });
  }
#endif

  if (!config_.no_audio_device) {
    webrtc::AudioOptions ao;
    if (config_.disable_echo_cancellation)
      ao.echo_cancellation = false;
    if (config_.disable_auto_gain_control)
      ao.auto_gain_control = false;
    if (config_.disable_noise_suppression)
      ao.noise_suppression = false;
    if (config_.disable_highpass_filter)
      ao.highpass_filter = false;
    RTC_LOG(LS_INFO) << __FUNCTION__ << ": " << ao.ToString();
    audio_track_ = factory_->CreateAudioTrack(
        Util::GenerateRandomChars(), factory_->CreateAudioSource(ao).get());
    if (!audio_track_) {
      RTC_LOG(LS_WARNING) << __FUNCTION__ << ": Cannot create audio_track";
    }
  }

  if (video_track_source && !config_.no_video_device) {
    webrtc::scoped_refptr<webrtc::VideoTrackSourceInterface> video_source =
        webrtc::VideoTrackSourceProxy::Create(
            signaling_thread_.get(), worker_thread_.get(), video_track_source);
    video_track_ =
        factory_->CreateVideoTrack(video_source, Util::GenerateRandomChars());
    if (video_track_) {
      if (config_.fixed_resolution) {
        video_track_->set_content_hint(
            webrtc::VideoTrackInterface::ContentHint::kText);
      }
    } else {
      RTC_LOG(LS_WARNING) << __FUNCTION__ << ": Cannot create video_track";
    }
  }
}

RTCManager::~RTCManager() {
  config_.create_adm = nullptr;
  video_sender_ = nullptr;
  audio_track_ = nullptr;
  video_track_ = nullptr;
  context_ = nullptr;
  factory_ = nullptr;
  network_thread_->Stop();
  worker_thread_->Stop();
  signaling_thread_->Stop();

  webrtc::CleanupSSL();
}

void RTCManager::AddDataManager(std::shared_ptr<RTCDataManager> data_manager) {
  data_manager_dispatcher_.Add(data_manager);
}

class RawCryptString : public webrtc::revive::CryptStringImpl {
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
  CryptStringImpl* Copy() const override { return new RawCryptString(str_); }
  void CopyRawTo(std::vector<unsigned char>* dest) const override {
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
  dependencies.tls_cert_verifier =
      std::unique_ptr<webrtc::SSLCertificateVerifier>(
          new RTCSSLVerifier(config_.insecure));

  dependencies.allocator.reset(new webrtc::BasicPortAllocator(
      webrtc::CreateEnvironment(), context_->default_network_manager(),
      context_->default_socket_factory(), rtc_config.turn_customizer));
  dependencies.allocator->SetPortRange(
      rtc_config.port_allocator_config.min_port,
      rtc_config.port_allocator_config.max_port);
  dependencies.allocator->set_flags(rtc_config.port_allocator_config.flags);
  if (!config_.proxy_url.empty()) {
    RTC_LOG(LS_INFO) << "Set Proxy: type="
                     << webrtc::revive::ProxyToString(
                            webrtc::revive::PROXY_HTTPS)
                     << " url=" << config_.proxy_url
                     << " username=" << config_.proxy_username;
    webrtc::revive::ProxyInfo pi;
    pi.type = webrtc::revive::PROXY_HTTPS;
    URLParts parts;
    if (!URLParts::Parse(config_.proxy_url, parts)) {
      RTC_LOG(LS_ERROR) << "Failed to parse: proxy_url=" << config_.proxy_url;
      return nullptr;
    }
    pi.address = webrtc::SocketAddress(parts.host, std::stoi(parts.GetPort()));
    if (!config_.proxy_username.empty()) {
      pi.username = config_.proxy_username;
    }
    if (!config_.proxy_password.empty()) {
      pi.password =
          webrtc::revive::CryptString(RawCryptString(config_.proxy_password));
    }
    dependencies.allocator->set_proxy("WebRTC Native Client Momo", pi);
  }

  webrtc::RTCErrorOr<webrtc::scoped_refptr<webrtc::PeerConnectionInterface>>
      connection = factory_->CreatePeerConnectionOrError(
          rtc_config, std::move(dependencies));
  if (!connection.ok()) {
    RTC_LOG(LS_ERROR) << __FUNCTION__ << ": CreatePeerConnection failed";
    return nullptr;
  }

  return std::make_shared<RTCConnection>(sender, std::move(observer),
                                         connection.value());
}

void RTCManager::InitTracks(RTCConnection* conn,
                            const std::optional<std::string>& direction) {
  if (direction.has_value() && *direction != "sendrecv" &&
      *direction != "sendonly" && *direction != "recvonly") {
    RTC_LOG(LS_WARNING)
        << __FUNCTION__
        << ": direction must be nullopt, sendrecv, sendonly, or recvonly";
    return;
  }

  auto connection = conn->GetConnection();

  std::string stream_id = Util::GenerateRandomChars();

  // 未設定か sendonly か sendrecv の場合はトラックを追加
  if (!direction.has_value() || direction == "sendonly" ||
      direction == "sendrecv") {
    if (audio_track_) {
      webrtc::RTCErrorOr<webrtc::scoped_refptr<webrtc::RtpSenderInterface>>
          audio_sender = connection->AddTrack(audio_track_, {stream_id});
      if (!audio_sender.ok()) {
        RTC_LOG(LS_WARNING) << __FUNCTION__ << ": Cannot add audio_track_";
      }
    }

    if (video_track_) {
      webrtc::RTCErrorOr<webrtc::scoped_refptr<webrtc::RtpSenderInterface>>
          video_add_result = connection->AddTrack(video_track_, {stream_id});
      if (video_add_result.ok()) {
        video_sender_ = video_add_result.value();
      } else {
        RTC_LOG(LS_WARNING) << __FUNCTION__ << ": Cannot add video_track_";
      }
    }

    if (direction.has_value()) {
      webrtc::RtpTransceiverDirection transceiver_direction =
          *direction == "sendonly" ? webrtc::RtpTransceiverDirection::kSendOnly
                                   : webrtc::RtpTransceiverDirection::kSendRecv;
      // すべてのトランシーバーに direction を設定
      for (auto transceiver : connection->GetTransceivers()) {
        auto error = transceiver->SetDirectionWithError(transceiver_direction);
        if (!error.ok()) {
          RTC_LOG(LS_WARNING)
              << __FUNCTION__
              << ": Failed to set transceiver direction: " << error.message();
        }
      }
    }
  } else {  // recvonly
    // recvonly の場合はトラックなしでトランシーバーを作成
    webrtc::RtpTransceiverInit init;
    init.direction = webrtc::RtpTransceiverDirection::kRecvOnly;

    // Audio トランシーバーを作成
    auto audio_result =
        connection->AddTransceiver(webrtc::MediaType::AUDIO, init);
    if (!audio_result.ok()) {
      RTC_LOG(LS_WARNING) << __FUNCTION__
                          << ": Cannot add audio transceiver for recvonly";
    }

    // Video トランシーバーを作成
    auto video_result =
        connection->AddTransceiver(webrtc::MediaType::VIDEO, init);
    if (!video_result.ok()) {
      RTC_LOG(LS_WARNING) << __FUNCTION__
                          << ": Cannot add video transceiver for recvonly";
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

webrtc::scoped_refptr<webrtc::PeerConnectionFactoryInterface>
RTCManager::GetFactory() const {
  return factory_;
}
