#include "audio_device_pipewire_linux.h"

#include <algorithm>
#include <chrono>
#include <thread>

#include <rtc_base/logging.h>

namespace webrtc {

AudioDeviceLinuxPipeWire::AudioDeviceLinuxPipeWire()
    : audio_buffer_(nullptr),
      audio_transport_(nullptr),
      initialized_(false),
      recording_(false),
      playing_(false),
      rec_is_initialized_(false),
      play_is_initialized_(false),
      input_device_index_(0),
      output_device_index_(0),
      input_device_is_specified_(false),
      output_device_is_specified_(false),
      pw_main_loop_(nullptr),
      pw_context_(nullptr),
      pw_core_(nullptr),
      pw_registry_(nullptr),
      play_stream_(nullptr),
      rec_stream_(nullptr) {
  RTC_LOG(LS_INFO) << __FUNCTION__;
}

AudioDeviceLinuxPipeWire::~AudioDeviceLinuxPipeWire() {
  RTC_LOG(LS_INFO) << __FUNCTION__;
  Terminate();
}

int32_t AudioDeviceLinuxPipeWire::ActiveAudioLayer(
    AudioDeviceModule::AudioLayer& audioLayer) const {
  audioLayer = AudioDeviceModule::kLinuxPulseAudio;  // PipeWire uses same enum
  return 0;
}

int32_t AudioDeviceLinuxPipeWire::RegisterAudioCallback(
    AudioTransport* audioCallback) {
  RTC_LOG(LS_INFO) << __FUNCTION__;
  audio_transport_ = audioCallback;

  // audio_buffer_ が既に attach されている場合は登録
  if (audio_buffer_ && audio_transport_) {
    audio_buffer_->RegisterAudioCallback(audio_transport_);
    RTC_LOG(LS_INFO) << "AudioCallback registered with audio_buffer_";
  }

  return 0;
}

AudioDeviceGeneric::InitStatus AudioDeviceLinuxPipeWire::Init() {
  RTC_LOG(LS_INFO) << __FUNCTION__;

  if (initialized_) {
    return InitStatus::OK;
  }

  // Initialize PipeWire
  pw_init(nullptr, nullptr);

  // Create thread loop
  pw_main_loop_ = pw_thread_loop_new("momo-audio", nullptr);
  if (!pw_main_loop_) {
    RTC_LOG(LS_ERROR) << "Failed to create PipeWire thread loop";
    return InitStatus::OTHER_ERROR;
  }

  // Create context
  pw_context_ =
      pw_context_new(pw_thread_loop_get_loop(pw_main_loop_), nullptr, 0);
  if (!pw_context_) {
    RTC_LOG(LS_ERROR) << "Failed to create PipeWire context";
    pw_thread_loop_destroy(pw_main_loop_);
    pw_main_loop_ = nullptr;
    return InitStatus::OTHER_ERROR;
  }

  // Start loop
  if (pw_thread_loop_start(pw_main_loop_) < 0) {
    RTC_LOG(LS_ERROR) << "Failed to start PipeWire thread loop";
    pw_context_destroy(pw_context_);
    pw_thread_loop_destroy(pw_main_loop_);
    pw_context_ = nullptr;
    pw_main_loop_ = nullptr;
    return InitStatus::OTHER_ERROR;
  }

  // Lock for thread safety
  pw_thread_loop_lock(pw_main_loop_);

  // Connect to PipeWire
  pw_core_ = pw_context_connect(pw_context_, nullptr, 0);
  if (!pw_core_) {
    RTC_LOG(LS_ERROR) << "Failed to connect to PipeWire";
    pw_thread_loop_unlock(pw_main_loop_);
    pw_thread_loop_stop(pw_main_loop_);
    pw_context_destroy(pw_context_);
    pw_thread_loop_destroy(pw_main_loop_);
    pw_core_ = nullptr;
    pw_context_ = nullptr;
    pw_main_loop_ = nullptr;
    return InitStatus::OTHER_ERROR;
  }

  // Enumerate devices
  EnumerateDevices();

  pw_thread_loop_unlock(pw_main_loop_);

  initialized_ = true;
  return InitStatus::OK;
}

int32_t AudioDeviceLinuxPipeWire::Terminate() {
  RTC_LOG(LS_INFO) << __FUNCTION__;

  if (!initialized_) {
    return 0;
  }

  StopRecording();
  StopPlayout();

  if (pw_main_loop_) {
    pw_thread_loop_stop(pw_main_loop_);
  }

  if (rec_stream_) {
    pw_stream_destroy(rec_stream_);
    rec_stream_ = nullptr;
  }

  if (play_stream_) {
    pw_stream_destroy(play_stream_);
    play_stream_ = nullptr;
  }

  if (pw_registry_) {
    pw_proxy_destroy((struct pw_proxy*)pw_registry_);
    pw_registry_ = nullptr;
  }

  if (pw_core_) {
    pw_core_disconnect(pw_core_);
    pw_core_ = nullptr;
  }

  if (pw_context_) {
    pw_context_destroy(pw_context_);
    pw_context_ = nullptr;
  }

  if (pw_main_loop_) {
    pw_thread_loop_destroy(pw_main_loop_);
    pw_main_loop_ = nullptr;
  }

  pw_deinit();

  initialized_ = false;
  return 0;
}

bool AudioDeviceLinuxPipeWire::Initialized() const {
  return initialized_;
}

void AudioDeviceLinuxPipeWire::EnumerateDevices() {
  RTC_LOG(LS_INFO) << __FUNCTION__;

  input_devices_.clear();
  output_devices_.clear();

  static const struct pw_registry_events registry_events = {
      .version = PW_VERSION_REGISTRY_EVENTS,
      .global = OnRegistryGlobal,
      .global_remove = OnRegistryGlobalRemove,
  };

  pw_registry_ = pw_core_get_registry(pw_core_, PW_VERSION_REGISTRY, 0);
  spa_zero(registry_listener_);
  pw_registry_add_listener(pw_registry_, &registry_listener_, &registry_events,
                           this);

  // Sync and wait for enumeration
  pw_core_sync(pw_core_, 0, 0);

  // Unlock to allow event processing
  pw_thread_loop_unlock(pw_main_loop_);
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  pw_thread_loop_lock(pw_main_loop_);
}

void AudioDeviceLinuxPipeWire::OnRegistryGlobal(
    void* data,
    uint32_t id,
    uint32_t permissions,
    const char* type,
    uint32_t version,
    const struct spa_dict* props) {
  auto* self = static_cast<AudioDeviceLinuxPipeWire*>(data);

  if (strcmp(type, PW_TYPE_INTERFACE_Node) != 0) {
    return;
  }

  const char* media_class = spa_dict_lookup(props, PW_KEY_MEDIA_CLASS);
  if (!media_class) {
    return;
  }

  const char* node_name = spa_dict_lookup(props, PW_KEY_NODE_NAME);
  const char* node_desc = spa_dict_lookup(props, PW_KEY_NODE_DESCRIPTION);

  DeviceInfo info;
  info.id = id;
  info.name = node_name ? node_name : "";
  info.description = node_desc ? node_desc : info.name;

  if (strcmp(media_class, "Audio/Source") == 0 ||
      strcmp(media_class, "Audio/Source/Virtual") == 0) {
    self->input_devices_.push_back(info);
    RTC_LOG(LS_INFO) << "Found input device: " << info.description << " (id="
                     << id << ")";
  } else if (strcmp(media_class, "Audio/Sink") == 0) {
    self->output_devices_.push_back(info);
    RTC_LOG(LS_INFO) << "Found output device: " << info.description << " (id="
                     << id << ")";
  }
}

void AudioDeviceLinuxPipeWire::OnRegistryGlobalRemove(void* data, uint32_t id) {
  // Handle device removal if needed
}

int16_t AudioDeviceLinuxPipeWire::PlayoutDevices() {
  return static_cast<int16_t>(output_devices_.size());
}

int16_t AudioDeviceLinuxPipeWire::RecordingDevices() {
  return static_cast<int16_t>(input_devices_.size());
}

int32_t AudioDeviceLinuxPipeWire::PlayoutDeviceName(
    uint16_t index,
    char name[kAdmMaxDeviceNameSize],
    char guid[kAdmMaxGuidSize]) {
  if (index >= output_devices_.size()) {
    return -1;
  }

  const auto& device = output_devices_[index];
  strncpy(name, device.description.c_str(), kAdmMaxDeviceNameSize - 1);
  name[kAdmMaxDeviceNameSize - 1] = '\0';

  if (guid) {
    strncpy(guid, device.name.c_str(), kAdmMaxGuidSize - 1);
    guid[kAdmMaxGuidSize - 1] = '\0';
  }

  return 0;
}

int32_t AudioDeviceLinuxPipeWire::RecordingDeviceName(
    uint16_t index,
    char name[kAdmMaxDeviceNameSize],
    char guid[kAdmMaxGuidSize]) {
  if (index >= input_devices_.size()) {
    return -1;
  }

  const auto& device = input_devices_[index];
  strncpy(name, device.description.c_str(), kAdmMaxDeviceNameSize - 1);
  name[kAdmMaxDeviceNameSize - 1] = '\0';

  if (guid) {
    strncpy(guid, device.name.c_str(), kAdmMaxGuidSize - 1);
    guid[kAdmMaxGuidSize - 1] = '\0';
  }

  return 0;
}

int32_t AudioDeviceLinuxPipeWire::SetPlayoutDevice(uint16_t index) {
  if (index >= output_devices_.size()) {
    return -1;
  }
  output_device_index_ = index;
  output_device_is_specified_ = true;
  return 0;
}

int32_t AudioDeviceLinuxPipeWire::SetPlayoutDevice(
    AudioDeviceModule::WindowsDeviceType device) {
  output_device_is_specified_ = false;
  return 0;
}

int32_t AudioDeviceLinuxPipeWire::SetRecordingDevice(uint16_t index) {
  if (index >= input_devices_.size()) {
    return -1;
  }
  input_device_index_ = index;
  input_device_is_specified_ = true;
  return 0;
}

int32_t AudioDeviceLinuxPipeWire::SetRecordingDevice(
    AudioDeviceModule::WindowsDeviceType device) {
  input_device_is_specified_ = false;
  return 0;
}

int32_t AudioDeviceLinuxPipeWire::PlayoutIsAvailable(bool& available) {
  available = !output_devices_.empty();
  return 0;
}

int32_t AudioDeviceLinuxPipeWire::InitPlayout() {
  RTC_LOG(LS_INFO) << __FUNCTION__;
  if (play_is_initialized_) {
    return 0;
  }

  if (!pw_main_loop_ || !pw_core_) {
    RTC_LOG(LS_ERROR) << "PipeWire not initialized";
    return -1;
  }

  pw_thread_loop_lock(pw_main_loop_);

  // Get target device ID
  uint32_t target_node_id = 0;
  if (output_device_is_specified_ && output_device_index_ < output_devices_.size()) {
    target_node_id = output_devices_[output_device_index_].id;
    RTC_LOG(LS_INFO) << "Using specified output device: "
                     << output_devices_[output_device_index_].description
                     << " (id=" << target_node_id << ")";
  } else if (!output_devices_.empty()) {
    target_node_id = output_devices_[0].id;
    RTC_LOG(LS_INFO) << "Using default output device: "
                     << output_devices_[0].description << " (id=" << target_node_id
                     << ")";
  } else {
    RTC_LOG(LS_ERROR) << "No output devices available";
    pw_thread_loop_unlock(pw_main_loop_);
    return -1;
  }

  // Create playout stream
  static const struct pw_stream_events play_stream_events = {
      .version = PW_VERSION_STREAM_EVENTS,
      .state_changed = OnPlayStreamStateChanged,
      .process = OnPlayStreamProcess,
  };

  // 10ms バッファを強制 (480 frames at 48kHz)
  // PW_KEY_NODE_FORCE_QUANTUM でシステム quantum を上書きする
  play_stream_ = pw_stream_new(pw_core_, "momo-playout",
                               pw_properties_new(PW_KEY_MEDIA_TYPE, "Audio",
                                                 PW_KEY_MEDIA_CATEGORY, "Playback",
                                                 PW_KEY_MEDIA_ROLE, "Communication",
                                                 PW_KEY_NODE_FORCE_QUANTUM, "480",
                                                 nullptr));
  if (!play_stream_) {
    RTC_LOG(LS_ERROR) << "Failed to create playout stream";
    pw_thread_loop_unlock(pw_main_loop_);
    return -1;
  }

  pw_stream_add_listener(play_stream_, &play_stream_listener_, &play_stream_events,
                         this);

  // Set up audio format and latency
  uint8_t buffer[1024];
  struct spa_pod_builder b = SPA_POD_BUILDER_INIT(buffer, sizeof(buffer));

  struct spa_audio_info_raw audio_info =
      SPA_AUDIO_INFO_RAW_INIT(.format = SPA_AUDIO_FORMAT_S16,
                               .rate = AudioDeviceLinuxPipeWire::kSampleRate,
                               .channels = AudioDeviceLinuxPipeWire::kChannels);

  const struct spa_pod* params[2];
  params[0] =
      spa_format_audio_raw_build(&b, SPA_PARAM_EnumFormat, &audio_info);

  // Latency を明示 (min/max を 480 フレームに固定して 10ms を強制)
  struct spa_latency_info latency_info = SPA_LATENCY_INFO(
      SPA_DIRECTION_OUTPUT,
      .min_quantum = kFramesPerBuffer,
      .max_quantum = kFramesPerBuffer);
  params[1] = spa_latency_build(&b, SPA_PARAM_Latency, &latency_info);

  // Connect to specific node
  char target_id[32];
  snprintf(target_id, sizeof(target_id), "%u", target_node_id);

  if (pw_stream_connect(play_stream_, PW_DIRECTION_OUTPUT, target_node_id,
                        static_cast<enum pw_stream_flags>(
                            PW_STREAM_FLAG_AUTOCONNECT | PW_STREAM_FLAG_MAP_BUFFERS |
                            PW_STREAM_FLAG_RT_PROCESS),
                        params, 2) < 0) {
    RTC_LOG(LS_ERROR) << "Failed to connect playout stream";
    pw_stream_destroy(play_stream_);
    play_stream_ = nullptr;
    pw_thread_loop_unlock(pw_main_loop_);
    return -1;
  }

  pw_thread_loop_unlock(pw_main_loop_);

  // AudioDeviceBuffer の初期化
  if (audio_buffer_) {
    audio_buffer_->SetPlayoutSampleRate(kSampleRate);
    audio_buffer_->SetPlayoutChannels(kChannels);
    RTC_LOG(LS_INFO) << "AudioDeviceBuffer configured: rate=" << kSampleRate
                     << " channels=" << kChannels;
  }

  // Playout buffer を初期化 (480 フレーム分)
  playout_buffer_size_ = kFramesPerBuffer * kChannels;
  playout_buffer_ = std::make_unique<int16_t[]>(playout_buffer_size_);
  playout_frames_in_buffer_ = 0;
  RTC_LOG(LS_INFO) << "Playout buffer initialized: " << kFramesPerBuffer
                   << " frames (" << playout_buffer_size_ << " samples)";

  play_is_initialized_ = true;
  RTC_LOG(LS_INFO) << "Playout stream initialized";
  return 0;
}

bool AudioDeviceLinuxPipeWire::PlayoutIsInitialized() const {
  return play_is_initialized_;
}

int32_t AudioDeviceLinuxPipeWire::RecordingIsAvailable(bool& available) {
  available = !input_devices_.empty();
  return 0;
}

int32_t AudioDeviceLinuxPipeWire::InitRecording() {
  RTC_LOG(LS_INFO) << __FUNCTION__;
  if (rec_is_initialized_) {
    return 0;
  }

  pw_thread_loop_lock(pw_main_loop_);

  // Get target device ID
  uint32_t target_node_id = 0;
  if (input_device_is_specified_ && input_device_index_ < input_devices_.size()) {
    target_node_id = input_devices_[input_device_index_].id;
    RTC_LOG(LS_INFO) << "Using specified input device: "
                     << input_devices_[input_device_index_].description
                     << " (id=" << target_node_id << ")";
  } else if (!input_devices_.empty()) {
    target_node_id = input_devices_[0].id;
    RTC_LOG(LS_INFO) << "Using default input device: "
                     << input_devices_[0].description << " (id=" << target_node_id
                     << ")";
  } else {
    RTC_LOG(LS_ERROR) << "No input devices available";
    pw_thread_loop_unlock(pw_main_loop_);
    return -1;
  }

  // Create recording stream
  static const struct pw_stream_events rec_stream_events = {
      .version = PW_VERSION_STREAM_EVENTS,
      .state_changed = OnRecStreamStateChanged,
      .process = OnRecStreamProcess,
  };

  // 10ms バッファを強制 (480 frames at 48kHz)
  // PW_KEY_NODE_FORCE_QUANTUM でシステム quantum を上書きする
  rec_stream_ = pw_stream_new(pw_core_, "momo-recording",
                               pw_properties_new(PW_KEY_MEDIA_TYPE, "Audio",
                                                 PW_KEY_MEDIA_CATEGORY, "Capture",
                                                 PW_KEY_MEDIA_ROLE, "Communication",
                                                 PW_KEY_NODE_FORCE_QUANTUM, "480",
                                                 nullptr));
  if (!rec_stream_) {
    RTC_LOG(LS_ERROR) << "Failed to create recording stream";
    pw_thread_loop_unlock(pw_main_loop_);
    return -1;
  }

  pw_stream_add_listener(rec_stream_, &rec_stream_listener_, &rec_stream_events,
                         this);

  // Set up audio format and latency
  uint8_t buffer[1024];
  struct spa_pod_builder b = SPA_POD_BUILDER_INIT(buffer, sizeof(buffer));

  struct spa_audio_info_raw audio_info =
      SPA_AUDIO_INFO_RAW_INIT(.format = SPA_AUDIO_FORMAT_S16,
                               .rate = AudioDeviceLinuxPipeWire::kSampleRate,
                               .channels = AudioDeviceLinuxPipeWire::kChannels);

  const struct spa_pod* params[2];
  params[0] =
      spa_format_audio_raw_build(&b, SPA_PARAM_EnumFormat, &audio_info);

  // Latency を明示 (min/max を 480 フレームに固定して 10ms を強制)
  struct spa_latency_info latency_info = SPA_LATENCY_INFO(
      SPA_DIRECTION_INPUT,
      .min_quantum = kFramesPerBuffer,
      .max_quantum = kFramesPerBuffer);
  params[1] = spa_latency_build(&b, SPA_PARAM_Latency, &latency_info);

  // Connect to specific node
  char target_id[32];
  snprintf(target_id, sizeof(target_id), "%u", target_node_id);

  if (pw_stream_connect(rec_stream_, PW_DIRECTION_INPUT, target_node_id,
                        static_cast<enum pw_stream_flags>(
                            PW_STREAM_FLAG_AUTOCONNECT | PW_STREAM_FLAG_MAP_BUFFERS |
                            PW_STREAM_FLAG_RT_PROCESS),
                        params, 2) < 0) {
    RTC_LOG(LS_ERROR) << "Failed to connect recording stream";
    pw_stream_destroy(rec_stream_);
    rec_stream_ = nullptr;
    pw_thread_loop_unlock(pw_main_loop_);
    return -1;
  }

  pw_thread_loop_unlock(pw_main_loop_);

  // AudioDeviceBuffer の初期化
  if (audio_buffer_) {
    audio_buffer_->SetRecordingSampleRate(kSampleRate);
    audio_buffer_->SetRecordingChannels(kChannels);
    RTC_LOG(LS_INFO) << "AudioDeviceBuffer configured: rate=" << kSampleRate
                     << " channels=" << kChannels;
  }

  // Recording buffer を初期化 (480 フレーム分)
  recording_buffer_size_ = kFramesPerBuffer * kChannels;
  recording_buffer_ = std::make_unique<int16_t[]>(recording_buffer_size_);
  recording_frames_in_buffer_ = 0;
  RTC_LOG(LS_INFO) << "Recording buffer initialized: " << kFramesPerBuffer
                   << " frames (" << recording_buffer_size_ << " samples)";

  rec_is_initialized_ = true;
  RTC_LOG(LS_INFO) << "Recording stream initialized";
  return 0;
}

bool AudioDeviceLinuxPipeWire::RecordingIsInitialized() const {
  return rec_is_initialized_;
}

int32_t AudioDeviceLinuxPipeWire::StartPlayout() {
  RTC_LOG(LS_INFO) << __FUNCTION__;
  if (!play_is_initialized_) {
    return -1;
  }
  if (playing_) {
    return 0;
  }

  pw_thread_loop_lock(pw_main_loop_);
  if (play_stream_) {
    pw_stream_set_active(play_stream_, true);
    RTC_LOG(LS_INFO) << "Playout stream activated";
  }
  pw_thread_loop_unlock(pw_main_loop_);

  playing_ = true;
  return 0;
}

int32_t AudioDeviceLinuxPipeWire::StopPlayout() {
  RTC_LOG(LS_INFO) << __FUNCTION__;
  if (!playing_) {
    return 0;
  }

  pw_thread_loop_lock(pw_main_loop_);
  if (play_stream_) {
    pw_stream_set_active(play_stream_, false);
    RTC_LOG(LS_INFO) << "Playout stream deactivated";
  }
  pw_thread_loop_unlock(pw_main_loop_);

  // バッファをリセットして、次回の再生開始時に前回の端数が混入しないようにする
  playout_frames_in_buffer_ = 0;

  playing_ = false;
  return 0;
}

bool AudioDeviceLinuxPipeWire::Playing() const {
  return playing_;
}

int32_t AudioDeviceLinuxPipeWire::StartRecording() {
  RTC_LOG(LS_INFO) << __FUNCTION__;
  if (!rec_is_initialized_) {
    return -1;
  }
  if (recording_) {
    return 0;
  }

  pw_thread_loop_lock(pw_main_loop_);
  if (rec_stream_) {
    pw_stream_set_active(rec_stream_, true);
    RTC_LOG(LS_INFO) << "Recording stream activated";
  }
  pw_thread_loop_unlock(pw_main_loop_);

  recording_ = true;
  return 0;
}

int32_t AudioDeviceLinuxPipeWire::StopRecording() {
  RTC_LOG(LS_INFO) << __FUNCTION__;
  if (!recording_) {
    return 0;
  }

  pw_thread_loop_lock(pw_main_loop_);
  if (rec_stream_) {
    pw_stream_set_active(rec_stream_, false);
    RTC_LOG(LS_INFO) << "Recording stream deactivated";
  }
  pw_thread_loop_unlock(pw_main_loop_);

  // バッファをリセットして、次回の録音開始時に前回の端数が混入しないようにする
  recording_frames_in_buffer_ = 0;

  recording_ = false;
  return 0;
}

bool AudioDeviceLinuxPipeWire::Recording() const {
  return recording_;
}

int32_t AudioDeviceLinuxPipeWire::PlayoutDelay(uint16_t& delayMS) const {
  delayMS = 0;  // TODO: Get actual delay
  return 0;
}

void AudioDeviceLinuxPipeWire::AttachAudioBuffer(AudioDeviceBuffer* audioBuffer) {
  RTC_LOG(LS_INFO) << "AttachAudioBuffer: " << audioBuffer;
  audio_buffer_ = audioBuffer;
  if (audio_buffer_ && audio_transport_) {
    audio_buffer_->RegisterAudioCallback(audio_transport_);
    RTC_LOG(LS_INFO) << "AudioCallback registered with audio_buffer_";
  }
}

void AudioDeviceLinuxPipeWire::OnRecStreamStateChanged(
    void* data,
    enum pw_stream_state old,
    enum pw_stream_state state,
    const char* error) {
  auto* self = static_cast<AudioDeviceLinuxPipeWire*>(data);
  RTC_LOG(LS_INFO) << "Recording stream state changed: " << old << " -> " << state;
  if (error) {
    RTC_LOG(LS_ERROR) << "Recording stream error: " << error;
  }
}

void AudioDeviceLinuxPipeWire::OnRecStreamProcess(void* data) {
  auto* self = static_cast<AudioDeviceLinuxPipeWire*>(data);

  static int callback_count = 0;
  static int frame_480_count = 0;
  static int frame_other_count = 0;
  static int frames_discarded = 0;

  if (callback_count++ % 500 == 0) {
    RTC_LOG(LS_INFO) << "OnRecStreamProcess stats: callbacks=" << callback_count
                     << " frame_480=" << frame_480_count
                     << " frame_other=" << frame_other_count
                     << " discarded=" << frames_discarded;
  }

  if (!self->audio_buffer_ || !self->recording_) {
    return;
  }

  struct pw_buffer* buf = pw_stream_dequeue_buffer(self->rec_stream_);
  if (!buf) {
    RTC_LOG(LS_WARNING) << "No buffer available in OnRecStreamProcess";
    return;
  }

  struct spa_buffer* spa_buf = buf->buffer;
  uint8_t* base_data = static_cast<uint8_t*>(spa_buf->datas[0].data);
  if (!base_data) {
    RTC_LOG(LS_WARNING) << "No sample data in buffer";
    pw_stream_queue_buffer(self->rec_stream_, buf);
    return;
  }

  // chunk の offset を考慮してサンプルポインタを計算
  uint32_t offset = spa_buf->datas[0].chunk->offset;
  uint32_t buffer_size = spa_buf->datas[0].chunk->size;
  int16_t* samples = reinterpret_cast<int16_t*>(base_data + offset);
  uint32_t n_frames = buffer_size / (sizeof(int16_t) * AudioDeviceLinuxPipeWire::kChannels);

  if (n_frames == 0) {
    RTC_LOG(LS_WARNING) << "Zero frames calculated from buffer_size=" << buffer_size;
    pw_stream_queue_buffer(self->rec_stream_, buf);
    return;
  }

  // 統計を更新
  if (n_frames == kFramesPerBuffer) {
    frame_480_count++;
  } else {
    frame_other_count++;
  }

  // PipeWire から受け取ったサンプルを recording_buffer に蓄積
  // 480 フレーム溜まったら WebRTC に配信
  uint32_t frames_to_process = n_frames;
  const int16_t* src_ptr = samples;

  while (frames_to_process > 0) {
    // バッファに入れられるフレーム数を計算
    uint32_t frames_available = kFramesPerBuffer - self->recording_frames_in_buffer_;
    uint32_t frames_to_copy = std::min(frames_to_process, frames_available);

    // バッファにコピー
    size_t dst_offset = self->recording_frames_in_buffer_ * kChannels;
    size_t samples_to_copy = frames_to_copy * kChannels;
    memcpy(&self->recording_buffer_[dst_offset], src_ptr,
           samples_to_copy * sizeof(int16_t));

    self->recording_frames_in_buffer_ += frames_to_copy;
    src_ptr += samples_to_copy;
    frames_to_process -= frames_to_copy;

    // バッファが 480 フレーム溜まったら WebRTC に配信
    if (self->recording_frames_in_buffer_ == kFramesPerBuffer) {
      // PipeWire のタイミング情報を取得 (配信直前に取得)
      struct pw_time time = {};
      int rec_delay_ms = 0;
      int play_delay_ms = 0;

      if (pw_stream_get_time_n(self->rec_stream_, &time, sizeof(time)) == 0) {
        if (time.rate.denom > 0 && time.rate.num > 0) {
          rec_delay_ms = static_cast<int>((time.delay * 1000LL * time.rate.num) / time.rate.denom);
        }
      }

      self->audio_buffer_->SetRecordedBuffer(self->recording_buffer_.get(),
                                            kFramesPerBuffer);
      self->audio_buffer_->SetVQEData(play_delay_ms, rec_delay_ms);
      self->audio_buffer_->DeliverRecordedData();

      self->recording_frames_in_buffer_ = 0;
    }
  }

  pw_stream_queue_buffer(self->rec_stream_, buf);
}

void AudioDeviceLinuxPipeWire::OnPlayStreamStateChanged(
    void* data,
    enum pw_stream_state old,
    enum pw_stream_state state,
    const char* error) {
  auto* self = static_cast<AudioDeviceLinuxPipeWire*>(data);
  RTC_LOG(LS_INFO) << "Playout stream state changed: " << old << " -> " << state;
  if (error) {
    RTC_LOG(LS_ERROR) << "Playout stream error: " << error;
  }
}

void AudioDeviceLinuxPipeWire::OnPlayStreamProcess(void* data) {
  auto* self = static_cast<AudioDeviceLinuxPipeWire*>(data);

  static int callback_count = 0;
  static int frame_480_count = 0;
  static int frame_other_count = 0;

  if (callback_count++ % 500 == 0) {
    RTC_LOG(LS_INFO) << "OnPlayStreamProcess stats: callbacks=" << callback_count
                     << " frame_480=" << frame_480_count
                     << " frame_other=" << frame_other_count;
  }

  if (!self->audio_buffer_ || !self->playing_) {
    return;
  }

  struct pw_buffer* buf = pw_stream_dequeue_buffer(self->play_stream_);
  if (!buf) {
    RTC_LOG(LS_WARNING) << "No buffer available in OnPlayStreamProcess";
    return;
  }

  struct spa_buffer* spa_buf = buf->buffer;
  uint8_t* base_data = static_cast<uint8_t*>(spa_buf->datas[0].data);
  if (!base_data) {
    RTC_LOG(LS_WARNING) << "No sample data in buffer";
    pw_stream_queue_buffer(self->play_stream_, buf);
    return;
  }

  // chunk の offset を考慮してサンプルポインタを計算
  uint32_t offset = spa_buf->datas[0].chunk->offset;
  uint32_t max_size = spa_buf->datas[0].maxsize;
  int16_t* samples = reinterpret_cast<int16_t*>(base_data + offset);
  uint32_t max_frames = (max_size - offset) / (sizeof(int16_t) * kChannels);

  // WebRTC から 10ms 単位でデータを取得してバッファに蓄積
  uint32_t frames_to_fill = std::min(max_frames, static_cast<uint32_t>(kFramesPerBuffer));

  // 統計を更新
  if (frames_to_fill == kFramesPerBuffer) {
    frame_480_count++;
  } else {
    frame_other_count++;
  }

  // playout_buffer から PipeWire バッファにコピー
  uint32_t frames_written = 0;
  int16_t* dst_ptr = samples;

  while (frames_written < frames_to_fill) {
    // バッファに残っているフレーム数を確認
    if (self->playout_frames_in_buffer_ == 0) {
      // バッファが空なら WebRTC からデータを取得
      self->audio_buffer_->RequestPlayoutData(kFramesPerBuffer);
      self->audio_buffer_->GetPlayoutData(self->playout_buffer_.get());
      self->playout_frames_in_buffer_ = kFramesPerBuffer;
    }

    // バッファから PipeWire にコピー
    uint32_t frames_available = self->playout_frames_in_buffer_;
    uint32_t frames_to_copy = std::min(frames_to_fill - frames_written, frames_available);

    size_t src_offset = (kFramesPerBuffer - self->playout_frames_in_buffer_) * kChannels;
    size_t samples_to_copy = frames_to_copy * kChannels;
    memcpy(dst_ptr, &self->playout_buffer_[src_offset],
           samples_to_copy * sizeof(int16_t));

    dst_ptr += samples_to_copy;
    frames_written += frames_to_copy;
    self->playout_frames_in_buffer_ -= frames_to_copy;
  }

  // chunk size を設定
  spa_buf->datas[0].chunk->size = frames_written * kChannels * sizeof(int16_t);
  spa_buf->datas[0].chunk->stride = kChannels * sizeof(int16_t);

  pw_stream_queue_buffer(self->play_stream_, buf);
}

}  // namespace webrtc
