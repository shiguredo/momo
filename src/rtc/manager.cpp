
#include <iostream>

#include "api/audio_codecs/builtin_audio_decoder_factory.h"
#include "api/audio_codecs/builtin_audio_encoder_factory.h"
#include "api/create_peerconnection_factory.h"
#include "modules/audio_device/include/audio_device.h"
#include "modules/audio_processing/include/audio_processing.h"
#include "modules/video_capture/video_capture.h"
#include "modules/video_capture/video_capture_factory.h"
#include "rtc_base/ssladapter.h"
#include "rtc_base/logging.h"

#include "capture_track_source.h"
#include "manager.h"
#include "observer.h"
#include "util.h"

#ifdef __APPLE__
#include "objc_codec_factory_helper.h"
#else
#include "api/video_codecs/builtin_video_decoder_factory.h"
#include "api/video_codecs/builtin_video_encoder_factory.h"
#endif

#if USE_ROS
#include "ros/ros_audio_device_module.h"
#endif

#if USE_IL_ENCODER
#include "api/video_codecs/builtin_video_decoder_factory.h"
#include "hwenc_il/il_encoder_factory.h"
#include "absl/memory/memory.h"
#endif

RTCManager::RTCManager(ConnectionSettings conn_settings,
                       std::unique_ptr<rtc::VideoSourceInterface<webrtc::VideoFrame>> capturer)
                       : _conn_settings(conn_settings)
{
  rtc::InitializeSSL();

  _networkThread = rtc::Thread::CreateWithSocketServer();
  _networkThread->Start();
  _workerThread = rtc::Thread::Create();
  _workerThread->Start();
  _signalingThread = rtc::Thread::Create();
  _signalingThread->Start();

  _factory = webrtc::CreatePeerConnectionFactory(
      _networkThread.get(), _workerThread.get(), _signalingThread.get(),

#if USE_ROS
      ROSAudioDeviceModule::Create(_conn_settings),
#elif __APPLE__
      webrtc::AudioDeviceModule::Create(0, webrtc::AudioDeviceModule::kPlatformDefaultAudio),
#else
      webrtc::AudioDeviceModule::Create(0, webrtc::AudioDeviceModule::kLinuxAlsaAudio),
#endif

      webrtc::CreateBuiltinAudioEncoderFactory(),
      webrtc::CreateBuiltinAudioDecoderFactory(),

#ifdef __APPLE__
      CreateObjCEncoderFactory(),
      CreateObjCDecoderFactory(),
#else
#if USE_IL_ENCODER
      std::unique_ptr<webrtc::VideoEncoderFactory>(absl::make_unique<ILVideoEncoderFactory>()),
#else
      webrtc::CreateBuiltinVideoEncoderFactory(),
#endif
      webrtc::CreateBuiltinVideoDecoderFactory(),
#endif

      nullptr, nullptr);
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

  if (capturer && !_conn_settings.no_video)
  {
    _video_source = CapturerTrackSource::Create(std::move(capturer));
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
  PeerConnectionObserver *observer = new PeerConnectionObserver(sender);
  rtc::scoped_refptr<webrtc::PeerConnectionInterface> connection = 
      _factory->CreatePeerConnection(
          rtc_config, nullptr, nullptr, observer);
  if (!connection)
  {
    RTC_LOG(LS_ERROR) << __FUNCTION__ << "CreatePeerConnection failed";
    return nullptr;
  }

  if (!_conn_settings.no_audio)
  {
    rtc::scoped_refptr<webrtc::AudioTrackInterface> audio_track(
        _factory->CreateAudioTrack(Util::generateRundomChars(), NULL));
    if (audio_track)
    {
      rtc::scoped_refptr<webrtc::RtpSenderInterface> audio_sender(
          connection->CreateSender(webrtc::MediaStreamTrackInterface::kAudioKind, Util::generateRundomChars()));
      audio_sender->SetTrack(audio_track);
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

      rtc::scoped_refptr<webrtc::RtpSenderInterface> video_sender(
          connection->CreateSender(webrtc::MediaStreamTrackInterface::kVideoKind, Util::generateRundomChars()));
      webrtc::RtpParameters parameters = video_sender->GetParameters();
      parameters.degradation_preference = _conn_settings.getPriority();
      video_sender->SetParameters(parameters);
      video_sender->SetTrack(video_track);
    } else {
      RTC_LOG(LS_WARNING) << __FUNCTION__ << "Cannot create video_track";
    }
  }

  return std::make_shared<RTCConnection>(sender, connection);
}
