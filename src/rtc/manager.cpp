
#include <iostream>

#include "api/test/fakeconstraints.h"
#include "api/audio_codecs/builtin_audio_decoder_factory.h"
#include "api/audio_codecs/builtin_audio_encoder_factory.h"
#include "modules/audio_device/include/audio_device.h"
#include "modules/audio_processing/include/audio_processing.h"
#include "modules/video_capture/video_capture_factory.h"
#include "media/engine/webrtcvideocapturerfactory.h"
#include "rtc_base/ssladapter.h"
#include "rtc_base/logging.h"

#include "manager.h"
#include "observer.h"
#include "util.h"

#ifdef __APPLE__
#include "objc_codec_factory_helper.h"
#else
#include "api/video_codecs/builtin_video_decoder_factory.h"
#include "api/video_codecs/builtin_video_encoder_factory.h"
#endif

#if USE_IL_ENCODER
#include "api/video_codecs/builtin_video_decoder_factory.h"
#include "hwenc_il/il_encoder_factory.h"
#include "absl/memory/memory.h"
#endif

RTCManager::RTCManager(ConnectionSettings conn_settings, std::unique_ptr<cricket::VideoCapturer> capturer) : _conn_settings(conn_settings)
{
  rtc::InitializeSSL();

  _networkThread = rtc::Thread::CreateWithSocketServer();
  _networkThread->Start();
  _workerThread = rtc::Thread::Create();
  _workerThread->Start();
  _signalingThread = rtc::Thread::Create();
  _signalingThread->Start();

#ifdef __APPLE__
  _adm = webrtc::AudioDeviceModule::Create(0, webrtc::AudioDeviceModule::kPlatformDefaultAudio);
#else
  _adm = webrtc::AudioDeviceModule::Create(0, webrtc::AudioDeviceModule::kLinuxAlsaAudio);
#endif

  _factory = webrtc::CreatePeerConnectionFactory(
      _networkThread.get(), _workerThread.get(), _signalingThread.get(),
      _adm,
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

  if (!_conn_settings.no_video)
  {
#if USE_ROS
    _video_source = _factory->CreateVideoSource(std::move(capturer));
#else

    capturer = createVideoCapturer(_conn_settings.video_device);
    if (capturer == nullptr)
    {
      exit(1);
    }

    webrtc::FakeConstraints constraints;
        constraints.AddMandatory(webrtc::MediaConstraintsInterface::kMaxWidth, _conn_settings.getWidth());
    constraints.AddMandatory(webrtc::MediaConstraintsInterface::kMaxHeight, _conn_settings.getHeight());
    constraints.AddOptional(webrtc::MediaConstraintsInterface::kMinWidth, _conn_settings.getWidth());
        constraints.AddOptional(webrtc::MediaConstraintsInterface::kMinHeight, _conn_settings.getHeight());
    constraints.AddOptional(webrtc::MediaConstraintsInterface::kMinWidth, _conn_settings.getWidth());
        constraints.AddOptional(webrtc::MediaConstraintsInterface::kMinHeight, _conn_settings.getHeight());
    if (_conn_settings.framerate != 0) {
      constraints.AddMandatory(webrtc::MediaConstraintsInterface::kMaxFrameRate, _conn_settings.framerate);
    }
    _video_source = _factory->CreateVideoSource(std::move(capturer), &constraints);
#endif
  }

  if (!_conn_settings.no_audio)
  {
    // 録音用デバイスの設定
    {
      const std::string& name = _conn_settings.recording_device;
      std::vector<std::string> device_names = listRecordingDevice();
      if (device_names.empty()) {
        RTC_LOG(LS_WARNING) << __FUNCTION__ << "No recording device";
      } else {
        int device_index = 0;

        // デバイス名が指定されていた場合、そのデバイスを探す
        if (!name.empty()) {
          auto it = std::find(device_names.begin(), device_names.end(), name);
          if (it == device_names.end()) {
            RTC_LOG(LS_ERROR) << "specified recording device '" << name << "' not found";
            exit(1);
          }
          device_index = std::distance(device_names.begin(), it);
        }

        _adm->SetRecordingDevice((int16_t)device_index);
      }
    }
    // 再生用デバイスの設定（必要になったら作る）
    {
    }
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

std::vector<std::string> RTCManager::listRecordingDevice() {
  std::vector<std::string> device_names;
  int num_devices = _adm->RecordingDevices();
  for (int i = 0; i < num_devices; i++) {
    char name[webrtc::kAdmMaxDeviceNameSize];
    char guid[webrtc::kAdmMaxGuidSize];
    if (_adm->RecordingDeviceName(i, name, guid) != -1) {
      RTC_LOG(LS_INFO) << "found recording device: " << name;
      device_names.push_back(name);
    }
  }
  return device_names;
}

std::vector<std::string> RTCManager::listPlayoutDevice() {
  std::vector<std::string> device_names;
  int num_devices = _adm->PlayoutDevices();
  for (int i = 0; i < num_devices; i++) {
    char name[webrtc::kAdmMaxDeviceNameSize];
    char guid[webrtc::kAdmMaxGuidSize];
    if (_adm->PlayoutDeviceName(i, name, guid) != -1) {
      RTC_LOG(LS_INFO) << "found playout device: " << name;
      device_names.push_back(name);
    }
  }
  return device_names;
}

std::vector<std::string> RTCManager::listVideoDevice() {
  std::vector<std::string> device_names;
  std::unique_ptr<webrtc::VideoCaptureModule::DeviceInfo> info(
            webrtc::VideoCaptureFactory::CreateDeviceInfo());
  if (!info) {
    RTC_LOG(LS_WARNING) << __FUNCTION__ << "CreateDeviceInfo failed";
    return {};
  }
  int num_devices = info->NumberOfDevices();
  for (int i = 0; i < num_devices; ++i) {
    const uint32_t nSize = 256;
    char name[nSize] = {0};
    char id[nSize] = {0};
    if (info->GetDeviceName(i, name, nSize, id, nSize) != -1) {
      RTC_LOG(LS_INFO) << "found video device: " << name;
      device_names.push_back(name);
    }
  }
  return device_names;
}

std::unique_ptr<cricket::VideoCapturer> RTCManager::createVideoCapturer(const std::string& name) {
  std::vector<std::string> device_names = listVideoDevice();
  if (device_names.empty()) {
    return nullptr;
  }

  int device_index = 0;

  // デバイス名が指定されていた場合、そのデバイスを探す
  if (!name.empty()) {
    auto it = std::find(device_names.begin(), device_names.end(), name);
    if (it == device_names.end()) {
      RTC_LOG(LS_ERROR) << "specified video device '" << name << "' not found";
      return nullptr;
    }
    device_index = std::distance(device_names.begin(), it);
  }

  cricket::WebRtcVideoDeviceCapturerFactory factory;
  return factory.Create(cricket::Device(device_names[device_index], 0));
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
