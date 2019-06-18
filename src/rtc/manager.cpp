
#include <iostream>

#include "api/audio_codecs/builtin_audio_decoder_factory.h"
#include "api/audio_codecs/builtin_audio_encoder_factory.h"
#include "api/create_peerconnection_factory.h"
#include "api/task_queue/global_task_queue_factory.h"
#include "api/video_track_source_proxy.h"
#include "logging/rtc_event_log/rtc_event_log_factory.h"
#include "media/engine/webrtc_media_engine.h"
#include "modules/audio_device/include/audio_device.h"
#include "modules/audio_processing/include/audio_processing.h"
#include "modules/video_capture/video_capture.h"
#include "modules/video_capture/video_capture_factory.h"
#include "rtc_base/ssl_adapter.h"
#include "rtc_base/logging.h"

#include "scalable_track_source.h"
#include "manager.h"
#include "observer.h"
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

#if USE_MMAL_ENCODER
#include "hw_video_encoder_factory.h"
#include "api/video_codecs/builtin_video_decoder_factory.h"
#include "absl/memory/memory.h"
#endif

RTCManager::RTCManager(ConnectionSettings conn_settings,
                       rtc::scoped_refptr<ScalableVideoTrackSource> video_track_source)
                       : _conn_settings(conn_settings)
{
  rtc::InitializeSSL();

  _networkThread = rtc::Thread::CreateWithSocketServer();
  _networkThread->Start();
  _workerThread = rtc::Thread::Create();
  _workerThread->Start();
  _signalingThread = rtc::Thread::Create();
  _signalingThread->Start();

#if __linux__
  webrtc::AudioDeviceModule::AudioLayer audio_layer = webrtc::AudioDeviceModule::kLinuxAlsaAudio;
#else
  webrtc::AudioDeviceModule::AudioLayer audio_layer = webrtc::AudioDeviceModule::kPlatformDefaultAudio;
#endif
  if (_conn_settings.no_audio)
  {
    audio_layer = webrtc::AudioDeviceModule::kDummyAudio;
  }

  std::unique_ptr<cricket::MediaEngineInterface> media_engine = cricket::WebRtcMediaEngineFactory::Create(
#if USE_ROS
      ROSAudioDeviceModule::Create(_conn_settings, &webrtc::GlobalTaskQueueFactory()),
#else
	  webrtc::AudioDeviceModule::Create(audio_layer, &webrtc::GlobalTaskQueueFactory()),
#endif
      webrtc::CreateBuiltinAudioEncoderFactory(),
      webrtc::CreateBuiltinAudioDecoderFactory(),
#ifdef __APPLE__
      CreateObjCEncoderFactory(),
      CreateObjCDecoderFactory(),
#else
#if USE_MMAL_ENCODER
      std::unique_ptr<webrtc::VideoEncoderFactory>(absl::make_unique<HWVideoEncoderFactory>()),
#else
      webrtc::CreateBuiltinVideoEncoderFactory(),
#endif
      webrtc::CreateBuiltinVideoDecoderFactory(),
#endif
      nullptr /* audio_mixer */,
      webrtc::AudioProcessingBuilder().Create());

  _factory = webrtc::CreateModularPeerConnectionFactory(
        _networkThread.get(), _workerThread.get(), _signalingThread.get(),
        std::move(media_engine), webrtc::CreateCallFactory(), webrtc::CreateRtcEventLogFactory());
  if (!_factory.get())
  {
    RTC_LOG(LS_ERROR) << __FUNCTION__ << "Failed to initialize PeerConnectionFactory";
    exit(1);
  }

  webrtc::PeerConnectionFactoryInterface::Options factory_options;
  factory_options.disable_sctp_data_channels = false;
  factory_options.disable_encryption = false;
  factory_options.ssl_max_version = rtc::SSL_PROTOCOL_DTLS_12;
  _factory->SetOptions(factory_options);

  if (video_track_source && !_conn_settings.no_video)
  {
    _video_source = webrtc::VideoTrackSourceProxy::Create(
          _signalingThread.get(), _workerThread.get(), video_track_source);
  }
}

RTCManager::~RTCManager()
{
  _video_source = NULL;
  _factory = NULL;
  _networkThread->Stop();
  _workerThread->Stop();
  _signalingThread->Stop();

  rtc::CleanupSSL();
}

std::shared_ptr<RTCConnection> RTCManager::createConnection(
        webrtc::PeerConnectionInterface::RTCConfiguration rtc_config,
        RTCMessageSender *sender)
{
  rtc_config.enable_dtls_srtp = true;
  rtc_config.sdp_semantics = webrtc::SdpSemantics::kUnifiedPlan;
  PeerConnectionObserver *observer = new PeerConnectionObserver(sender);
  rtc::scoped_refptr<webrtc::PeerConnectionInterface> connection = 
      _factory->CreatePeerConnection(
          rtc_config, nullptr, nullptr, observer);
  if (!connection)
  {
    RTC_LOG(LS_ERROR) << __FUNCTION__ << "CreatePeerConnection failed";
    return nullptr;
  }

  std::string stream_id = Util::generateRundomChars();

  if (!_conn_settings.no_audio)
  {
    rtc::scoped_refptr<webrtc::AudioTrackInterface> audio_track(
        _factory->CreateAudioTrack(Util::generateRundomChars(),
                                   _factory->CreateAudioSource(cricket::AudioOptions())));
    if (audio_track)
    {
      webrtc::RTCErrorOr<rtc::scoped_refptr<webrtc::RtpSenderInterface> > audio_sender =
          connection->AddTrack(audio_track, {stream_id});
      if (!audio_sender.ok())
      {
        RTC_LOG(LS_WARNING) << __FUNCTION__ << "Cannot add audio_track";
      }
    } else {
      RTC_LOG(LS_WARNING) << __FUNCTION__ << "Cannot create audio_track";
    }
  }

  if (_video_source) {
    rtc::scoped_refptr<webrtc::VideoTrackInterface> video_track(
            _factory->CreateVideoTrack(Util::generateRundomChars(), _video_source));
    if (video_track)
    {
      if (_conn_settings.fixed_resolution) {
        video_track->set_content_hint(webrtc::VideoTrackInterface::ContentHint::kText);
      }

      webrtc::RTCErrorOr<rtc::scoped_refptr<webrtc::RtpSenderInterface> > video_add_result =
          connection->AddTrack(video_track, {stream_id});
      if (video_add_result.ok())
      {
        rtc::scoped_refptr<webrtc::RtpSenderInterface> video_sender = video_add_result.value();
        webrtc::RtpParameters parameters = video_sender->GetParameters();
        parameters.degradation_preference = _conn_settings.getPriority();
        video_sender->SetParameters(parameters);
      } else {
        RTC_LOG(LS_WARNING) << __FUNCTION__ << "Cannot add video_track";
      }
    } else {
      RTC_LOG(LS_WARNING) << __FUNCTION__ << "Cannot create video_track";
    }
  }

  return std::make_shared<RTCConnection>(sender, connection);
}
