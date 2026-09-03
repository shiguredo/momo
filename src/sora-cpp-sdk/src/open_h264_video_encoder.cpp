/*
 *  Copyright (c) 2015 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 *
 */

// modules/video_coding/codecs/h264/h264_encoder_impl.{h,cc} の
// OpenH264 の関数を動的に読むようにしただけ

#include "sora/open_h264_video_encoder.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#if defined(_WIN32)
// Windows
#include <windows.h>
#else
// Linux
#include <dlfcn.h>
#endif

// WebRTC
#include <absl/container/inlined_vector.h>
#include <absl/memory/memory.h>
#include <api/environment/environment.h>
#include <api/environment/environment_factory.h>
#include <api/scoped_refptr.h>
#include <api/units/data_rate.h>
#include <api/video/encoded_image.h>
#include <api/video/i420_buffer.h>
#include <api/video/video_bitrate_allocation.h>
#include <api/video/video_bitrate_allocator.h>
#include <api/video/video_codec_constants.h>
#include <api/video/video_codec_type.h>
#include <api/video/video_frame.h>
#include <api/video/video_frame_buffer.h>
#include <api/video/video_frame_type.h>
#include <api/video_codecs/scalability_mode.h>
#include <api/video_codecs/sdp_video_format.h>
#include <api/video_codecs/video_codec.h>
#include <api/video_codecs/video_encoder.h>
#include <common_video/h264/h264_bitstream_parser.h>
#include <common_video/libyuv/include/webrtc_libyuv.h>
#include <media/base/media_constants.h>
#include <modules/video_coding/codecs/h264/include/h264.h>
#include <modules/video_coding/codecs/h264/include/h264_globals.h>
#include <modules/video_coding/codecs/interface/common_constants.h>
#include <modules/video_coding/include/video_codec_interface.h>
#include <modules/video_coding/include/video_error_codes.h>
#include <modules/video_coding/svc/create_scalability_structure.h>
#include <modules/video_coding/svc/scalable_video_controller.h>
#include <modules/video_coding/utility/simulcast_rate_allocator.h>
#include <modules/video_coding/utility/simulcast_utility.h>
#include <rtc_base/checks.h>
#include <rtc_base/logging.h>
#include <system_wrappers/include/metrics.h>

// libyuv
#include <libyuv/scale.h>

// OpenH264
#include <wels/codec_api.h>
#include <wels/codec_app_def.h>
#include <wels/codec_def.h>
#include <wels/codec_ver.h>

class ISVCEncoder;

namespace webrtc {

class OpenH264VideoEncoder : public VideoEncoder {
 public:
  struct LayerConfig {
    int simulcast_idx = 0;
    int width = -1;
    int height = -1;
    bool sending = true;
    bool key_frame_request = false;
    float max_frame_rate = 0;
    uint32_t target_bps = 0;
    uint32_t max_bps = 0;
    bool frame_dropping_on = false;
    int key_frame_interval = 0;
    int num_temporal_layers = 1;

    void SetStreamState(bool send_stream);
  };

  OpenH264VideoEncoder(const Environment& env,
                       H264EncoderSettings settings,
                       std::string openh264);

  ~OpenH264VideoEncoder() override;

  // `settings.max_payload_size` は無視する。
  // `codec_settings` では次のメンバだけ使う。それ以外は無視する。
  // - codecType (kVideoCodecH264 であること)
  // - targetBitrate
  // - maxFramerate
  // - width
  // - height
  int32_t InitEncode(const VideoCodec* codec_settings,
                     const VideoEncoder::Settings& settings) override;
  int32_t Release() override;

  int32_t RegisterEncodeCompleteCallback(
      EncodedImageCallback* callback) override;
  void SetRates(const RateControlParameters& parameters) override;

  // エンコード結果 (EncodedImage と CodecSpecificInfo) を
  // encode complete コールバックへ渡す。
  int32_t Encode(const VideoFrame& frame,
                 const std::vector<VideoFrameType>* frame_types) override;

  EncoderInfo GetEncoderInfo() const override;

  // テスト用に公開する。
  H264PacketizationMode PacketizationModeForTesting() const {
    return packetization_mode_;
  }

 private:
  SEncParamExt CreateEncoderParams(size_t i) const;

  webrtc::H264BitstreamParser h264_bitstream_parser_;
  // ヒストグラムで統計を報告する。
  void ReportInit();
  void ReportError();

  std::vector<ISVCEncoder*> encoders_;
  std::vector<SSourcePicture> pictures_;
  std::vector<webrtc::scoped_refptr<I420Buffer>> downscaled_buffers_;
  std::vector<LayerConfig> configurations_;
  std::vector<EncodedImage> encoded_images_;
  std::vector<std::unique_ptr<ScalableVideoController>> svc_controllers_;
  absl::InlinedVector<std::optional<ScalabilityMode>, kMaxSimulcastStreams>
      scalability_modes_;

  const Environment env_;
  VideoCodec codec_;
  H264PacketizationMode packetization_mode_;
  size_t max_payload_size_;
  int32_t number_of_cores_;
  std::optional<int> encoder_thread_limit_;
  EncodedImageCallback* encoded_image_callback_;

  bool has_reported_init_;
  bool has_reported_error_;

  std::vector<uint8_t> tl0sync_limit_;

 private:
  bool InitOpenH264();
  void ReleaseOpenH264();

  std::string openh264_;
#if defined(_WIN32)
  HMODULE openh264_handle_ = nullptr;
#else
  void* openh264_handle_ = nullptr;
#endif
  using CreateEncoderFunc = int (*)(ISVCEncoder**);
  using DestroyEncoderFunc = void (*)(ISVCEncoder*);
  CreateEncoderFunc create_encoder_ = nullptr;
  DestroyEncoderFunc destroy_encoder_ = nullptr;
};

}  // namespace webrtc

namespace webrtc {

namespace {

const bool kOpenH264EncoderDetailedLogging = false;

// QP スケーリングの閾値。
static const int kLowH264QpThreshold = 24;
static const int kHighH264QpThreshold = 37;

// ヒストグラム用。エントリの値は変えてはならない。
enum H264EncoderImplEvent {
  kH264EncoderEventInit = 0,
  kH264EncoderEventError = 1,
  kH264EncoderEventMax = 16,
};

int NumberOfThreads(std::optional<int> encoder_thread_limit,
                    int width,
                    int height,
                    int number_of_cores) {
  // TODO(hbos): Chromium の Mac サンドボックスでは複数スレッドが動かない。
  // crbug.com/583348 を参照。調査が終わるまでスレッドは 1 本にする。
  // 制限自体は解消済みだが、ビットストリーム形式が変わる (bugs.webrtc.org/14368)。
  // 実験用 field trial WebRTC-VideoEncoderSettings/encoder_thread_limit でガードする。
  if (encoder_thread_limit.has_value()) {
    int limit = encoder_thread_limit.value();
    RTC_DCHECK_GE(limit, 1);
    if (width * height >= 1920 * 1080 && number_of_cores > 8) {
      return std::min(limit, 8);  // 高性能機の 1080p では 8 スレッド。
    } else if (width * height > 1280 * 960 && number_of_cores >= 6) {
      return std::min(limit, 3);  // 1080p では 3 スレッド。
    } else if (width * height > 640 * 480 && number_of_cores >= 3) {
      return std::min(limit, 2);  // qHD/HD では 2 スレッド。
    } else {
      return 1;  // VGA 以下では 1 スレッド。
    }
  }
  // TODO(sprang): ここでマルチスレッドを有効にする前に、
  // GetEncoderParams() の sSliceArgument.uiSliceNum も確認する。
  return 1;
}

VideoFrameType ConvertToVideoFrameType(EVideoFrameType type) {
  switch (type) {
    case videoFrameTypeIDR:
      return VideoFrameType::kVideoFrameKey;
    case videoFrameTypeSkip:
    case videoFrameTypeI:
    case videoFrameTypeP:
    case videoFrameTypeIPMixed:
      return VideoFrameType::kVideoFrameDelta;
    case videoFrameTypeInvalid:
      break;
  }
  RTC_DCHECK_NOTREACHED() << "Unexpected/invalid frame type: " << type;
  return VideoFrameType::kEmptyFrame;
}

std::optional<ScalabilityMode> ScalabilityModeFromTemporalLayers(
    int num_temporal_layers) {
  switch (num_temporal_layers) {
    case 0:
      break;
    case 1:
      return ScalabilityMode::kL1T1;
    case 2:
      return ScalabilityMode::kL1T2;
    case 3:
      return ScalabilityMode::kL1T3;
    default:
      RTC_DCHECK_NOTREACHED();
  }
  return std::nullopt;
}

}  // namespace

// OpenH264VideoEncoder::Encode が使うヘルパ。
// `info` のエンコード済みバイトを `encoded_image` へコピーする。
// より大きいバッファが必要なら `encoded_image->_buffer` を破棄して再確保する。
//
// OpenH264 エンコード後、バイト列は複数レイヤーと NAL ユニットに分かれて `info` に入る。
// 各 NAL ユニットは 4 バイトのスタートコード {0,0,0,1} で始まるフラグメントである。
// スタートコードを含むこのデータをすべて `encoded_image->_buffer` へコピーする。
static void RtpFragmentize(EncodedImage* encoded_image, SFrameBSInfo* info) {
  // エンコードデータを収める最小バッファサイズを計算する。
  size_t required_capacity = 0;
  size_t fragments_count = 0;
  for (int layer = 0; layer < info->iLayerNum; ++layer) {
    const SLayerBSInfo& layerInfo = info->sLayerInfo[layer];
    for (int nal = 0; nal < layerInfo.iNalCount; ++nal, ++fragments_count) {
      RTC_CHECK_GE(layerInfo.pNalLengthInByte[nal], 0);
      // `required_capacity` がオーバーフローしないことを確認する。
      RTC_CHECK_LE(layerInfo.pNalLengthInByte[nal],
                   std::numeric_limits<size_t>::max() - required_capacity);
      required_capacity += layerInfo.pNalLengthInByte[nal];
    }
  }
  auto buffer = EncodedImageBuffer::Create(required_capacity);
  encoded_image->SetEncodedData(buffer);

  // レイヤーと NAL ユニットを順に見て、各 NAL をフラグメントとして記録し
  // `encoded_image->_buffer` へコピーする。
  const uint8_t start_code[4] = {0, 0, 0, 1};
  size_t frag = 0;
  encoded_image->set_size(0);
  for (int layer = 0; layer < info->iLayerNum; ++layer) {
    const SLayerBSInfo& layerInfo = info->sLayerInfo[layer];
    // このレイヤーを構成する NAL ユニットを順に見てフラグメントを記録する。
    size_t layer_len = 0;
    for (int nal = 0; nal < layerInfo.iNalCount; ++nal, ++frag) {
      // 全レイヤー長の合計 `required_capacity` は `size_t` に収まるので、
      // 途中のインデックスもオーバーフローしない。
      RTC_DCHECK_GE(layerInfo.pNalLengthInByte[nal], 4);
      RTC_DCHECK_EQ(layerInfo.pBsBuf[layer_len + 0], start_code[0]);
      RTC_DCHECK_EQ(layerInfo.pBsBuf[layer_len + 1], start_code[1]);
      RTC_DCHECK_EQ(layerInfo.pBsBuf[layer_len + 2], start_code[2]);
      RTC_DCHECK_EQ(layerInfo.pBsBuf[layer_len + 3], start_code[3]);
      layer_len += layerInfo.pNalLengthInByte[nal];
    }
    // スタートコードを含め、レイヤー全体のデータをコピーする。
    memcpy(buffer->data() + encoded_image->size(), layerInfo.pBsBuf, layer_len);
    encoded_image->set_size(encoded_image->size() + layer_len);
  }
}

OpenH264VideoEncoder::OpenH264VideoEncoder(const Environment& env,
                                           H264EncoderSettings settings,
                                           std::string openh264)
    : env_(env),
      packetization_mode_(settings.packetization_mode),
      max_payload_size_(0),
      number_of_cores_(0),
      encoded_image_callback_(nullptr),
      has_reported_init_(false),
      has_reported_error_(false),
      openh264_(std::move(openh264)) {
  downscaled_buffers_.reserve(kMaxSimulcastStreams - 1);
  encoded_images_.reserve(kMaxSimulcastStreams);
  encoders_.reserve(kMaxSimulcastStreams);
  configurations_.reserve(kMaxSimulcastStreams);
  tl0sync_limit_.reserve(kMaxSimulcastStreams);
  svc_controllers_.reserve(kMaxSimulcastStreams);
}

OpenH264VideoEncoder::~OpenH264VideoEncoder() {
  Release();
  ReleaseOpenH264();
}

bool OpenH264VideoEncoder::InitOpenH264() {
  if (openh264_handle_ != nullptr) {
    return true;
  }

#if defined(_WIN32)
  HMODULE handle = LoadLibraryA(openh264_.c_str());
#else
  void* handle = ::dlopen(openh264_.c_str(), RTLD_LAZY);
#endif
  if (handle == nullptr) {
    return false;
  }
#if defined(_WIN32)
  create_encoder_ =
      (CreateEncoderFunc)::GetProcAddress(handle, "WelsCreateSVCEncoder");
  if (create_encoder_ == nullptr) {
    FreeLibrary(handle);
    return false;
  }
  destroy_encoder_ =
      (DestroyEncoderFunc)::GetProcAddress(handle, "WelsDestroySVCEncoder");
  if (destroy_encoder_ == nullptr) {
    FreeLibrary(handle);
    return false;
  }
#else
  create_encoder_ = (CreateEncoderFunc)::dlsym(handle, "WelsCreateSVCEncoder");
  if (create_encoder_ == nullptr) {
    ::dlclose(handle);
    return false;
  }
  destroy_encoder_ =
      (DestroyEncoderFunc)::dlsym(handle, "WelsDestroySVCEncoder");
  if (destroy_encoder_ == nullptr) {
    ::dlclose(handle);
    return false;
  }
#endif
  openh264_handle_ = handle;
  return true;
}

void OpenH264VideoEncoder::ReleaseOpenH264() {
  if (openh264_handle_ != nullptr) {
#if defined(_WIN32)
    FreeLibrary(openh264_handle_);
#else
    ::dlclose(openh264_handle_);
#endif
    openh264_handle_ = nullptr;
  }
}

int32_t OpenH264VideoEncoder::InitEncode(
    const VideoCodec* inst,
    const VideoEncoder::Settings& settings) {
  ReportInit();
  if (!inst || inst->codecType != kVideoCodecH264) {
    ReportError();
    return WEBRTC_VIDEO_CODEC_ERR_PARAMETER;
  }
  if (inst->maxFramerate == 0) {
    ReportError();
    return WEBRTC_VIDEO_CODEC_ERR_PARAMETER;
  }
  if (inst->width < 1 || inst->height < 1) {
    ReportError();
    return WEBRTC_VIDEO_CODEC_ERR_PARAMETER;
  }

  int32_t release_ret = Release();
  if (release_ret != WEBRTC_VIDEO_CODEC_OK) {
    ReportError();
    return release_ret;
  }

  if (!InitOpenH264()) {
    ReportError();
    return WEBRTC_VIDEO_CODEC_ERROR;
  }

  int number_of_streams = SimulcastUtility::NumberOfSimulcastStreams(*inst);
  bool doing_simulcast = (number_of_streams > 1);

  if (doing_simulcast &&
      !SimulcastUtility::ValidSimulcastParameters(*inst, number_of_streams)) {
    return WEBRTC_VIDEO_CODEC_ERR_SIMULCAST_PARAMETERS_NOT_SUPPORTED;
  }
  downscaled_buffers_.resize(number_of_streams - 1);
  encoded_images_.resize(number_of_streams);
  encoders_.resize(number_of_streams);
  pictures_.resize(number_of_streams);
  svc_controllers_.resize(number_of_streams);
  scalability_modes_.resize(number_of_streams);
  configurations_.resize(number_of_streams);
  tl0sync_limit_.resize(number_of_streams);

  max_payload_size_ = settings.max_payload_size;
  number_of_cores_ = settings.number_of_cores;
  encoder_thread_limit_ = settings.encoder_thread_limit;
  codec_ = *inst;

  // コードは simulcastStream の解像度が正しいことを前提にする。
  // サイマルキャストレイヤーが無くても埋めておく。
  if (codec_.numberOfSimulcastStreams == 0) {
    codec_.simulcastStream[0].width = codec_.width;
    codec_.simulcastStream[0].height = codec_.height;
  }

  for (int i = 0, idx = number_of_streams - 1; i < number_of_streams;
       ++i, --idx) {
    ISVCEncoder* openh264_encoder;
    // エンコーダを作成する。
    if (create_encoder_(&openh264_encoder) != 0) {
      // エンコーダの作成に失敗した。
      RTC_LOG(LS_ERROR) << "Failed to create OpenH264 encoder";
      RTC_DCHECK(!openh264_encoder);
      Release();
      ReportError();
      return WEBRTC_VIDEO_CODEC_ERROR;
    }
    RTC_DCHECK(openh264_encoder);
    if (kOpenH264EncoderDetailedLogging) {
      int trace_level = WELS_LOG_DETAIL;
      openh264_encoder->SetOption(ENCODER_OPTION_TRACE_LEVEL, &trace_level);
    }
    // それ以外は既定の WELS_LOG_DEFAULT を使う。

    // H.264 エンコーダを保存する。
    encoders_[i] = openh264_encoder;

    // codec_settings から内部設定を入れる
    configurations_[i].simulcast_idx = idx;
    configurations_[i].sending = false;
    configurations_[i].width = codec_.simulcastStream[idx].width;
    configurations_[i].height = codec_.simulcastStream[idx].height;
    configurations_[i].max_frame_rate = static_cast<float>(codec_.maxFramerate);
    configurations_[i].frame_dropping_on = codec_.GetFrameDropEnabled();
    configurations_[i].key_frame_interval = codec_.H264()->keyFrameInterval;
    configurations_[i].num_temporal_layers =
        std::max(codec_.H264()->numberOfTemporalLayers,
                 codec_.simulcastStream[idx].numberOfTemporalLayers);

    // 縮小画像バッファを作成する。
    if (i > 0) {
      downscaled_buffers_[i - 1] = I420Buffer::Create(
          configurations_[i].width, configurations_[i].height,
          configurations_[i].width, configurations_[i].width / 2,
          configurations_[i].width / 2);
    }

    // codec_settings は kbit/s、エンコーダは bit/s。
    configurations_[i].max_bps = codec_.maxBitrate * 1000;
    configurations_[i].target_bps = codec_.startBitrate * 1000;

    // レイヤー設定からエンコーダパラメータを作る。
    SEncParamExt encoder_params = CreateEncoderParams(i);

    // 初期化する。
    if (openh264_encoder->InitializeExt(&encoder_params) != 0) {
      RTC_LOG(LS_ERROR) << "Failed to initialize OpenH264 encoder";
      Release();
      ReportError();
      return WEBRTC_VIDEO_CODEC_ERROR;
    }
    // TODO(pbos): 投入前にこれらの値を初期化パラメータの根拠にする。
    int video_format = EVideoFormatType::videoFormatI420;
    openh264_encoder->SetOption(ENCODER_OPTION_DATAFORMAT, &video_format);

    // EncodedImage を初期化する。既定バッファサイズは未エンコードデータのサイズとする。

    const size_t new_capacity =
        CalcBufferSize(VideoType::kI420, codec_.simulcastStream[idx].width,
                       codec_.simulcastStream[idx].height);
    encoded_images_[i].SetEncodedData(EncodedImageBuffer::Create(new_capacity));
    encoded_images_[i]._encodedWidth = codec_.simulcastStream[idx].width;
    encoded_images_[i]._encodedHeight = codec_.simulcastStream[idx].height;
    encoded_images_[i].set_size(0);

    tl0sync_limit_[i] = configurations_[i].num_temporal_layers;
    scalability_modes_[i] = ScalabilityModeFromTemporalLayers(
        configurations_[i].num_temporal_layers);
    if (scalability_modes_[i].has_value()) {
      svc_controllers_[i] = CreateScalabilityStructure(*scalability_modes_[i]);
      if (svc_controllers_[i] == nullptr) {
        RTC_LOG(LS_ERROR) << "Failed to create scalability structure";
        Release();
        ReportError();
        return WEBRTC_VIDEO_CODEC_ERROR;
      }
    }
  }

  SimulcastRateAllocator init_allocator(env_, codec_);
  VideoBitrateAllocation allocation =
      init_allocator.Allocate(VideoBitrateAllocationParameters(
          DataRate::KilobitsPerSec(codec_.startBitrate), codec_.maxFramerate));
  SetRates(RateControlParameters(allocation, codec_.maxFramerate));
  return WEBRTC_VIDEO_CODEC_OK;
}

int32_t OpenH264VideoEncoder::Release() {
  while (!encoders_.empty()) {
    ISVCEncoder* openh264_encoder = encoders_.back();
    if (openh264_encoder) {
      RTC_CHECK_EQ(0, openh264_encoder->Uninitialize());
      destroy_encoder_(openh264_encoder);
    }
    encoders_.pop_back();
  }
  downscaled_buffers_.clear();
  configurations_.clear();
  encoded_images_.clear();
  pictures_.clear();
  tl0sync_limit_.clear();
  svc_controllers_.clear();
  scalability_modes_.clear();
  return WEBRTC_VIDEO_CODEC_OK;
}

int32_t OpenH264VideoEncoder::RegisterEncodeCompleteCallback(
    EncodedImageCallback* callback) {
  encoded_image_callback_ = callback;
  return WEBRTC_VIDEO_CODEC_OK;
}

void OpenH264VideoEncoder::SetRates(const RateControlParameters& parameters) {
  if (encoders_.empty()) {
    RTC_LOG(LS_WARNING) << "SetRates() while uninitialized.";
    return;
  }

  if (parameters.framerate_fps < 1.0) {
    RTC_LOG(LS_WARNING) << "Invalid frame rate: " << parameters.framerate_fps;
    return;
  }

  if (parameters.bitrate.get_sum_bps() == 0) {
    // エンコーダが一時停止しているので、すべてのエンコードを止める。
    for (size_t i = 0; i < configurations_.size(); ++i) {
      configurations_[i].SetStreamState(false);
    }
    return;
  }

  codec_.maxFramerate = static_cast<uint32_t>(parameters.framerate_fps);

  size_t stream_idx = encoders_.size() - 1;
  for (size_t i = 0; i < encoders_.size(); ++i, --stream_idx) {
    // レイヤー設定を更新する。
    configurations_[i].target_bps =
        parameters.bitrate.GetSpatialLayerSum(stream_idx);
    configurations_[i].max_frame_rate = parameters.framerate_fps;

    if (configurations_[i].target_bps) {
      configurations_[i].SetStreamState(true);

      // H.264 エンコーダを更新する。
      SBitrateInfo target_bitrate;
      memset(&target_bitrate, 0, sizeof(SBitrateInfo));
      target_bitrate.iLayer = SPATIAL_LAYER_ALL,
      target_bitrate.iBitrate = configurations_[i].target_bps;
      encoders_[i]->SetOption(ENCODER_OPTION_BITRATE, &target_bitrate);
      encoders_[i]->SetOption(ENCODER_OPTION_FRAME_RATE,
                              &configurations_[i].max_frame_rate);
    } else {
      configurations_[i].SetStreamState(false);
    }
  }
}

int32_t OpenH264VideoEncoder::Encode(
    const VideoFrame& input_frame,
    const std::vector<VideoFrameType>* frame_types) {
  if (encoders_.empty()) {
    ReportError();
    return WEBRTC_VIDEO_CODEC_UNINITIALIZED;
  }
  if (!encoded_image_callback_) {
    RTC_LOG(LS_WARNING)
        << "InitEncode() has been called, but a callback function "
           "has not been set with RegisterEncodeCompleteCallback()";
    ReportError();
    return WEBRTC_VIDEO_CODEC_UNINITIALIZED;
  }

  webrtc::scoped_refptr<I420BufferInterface> frame_buffer =
      input_frame.video_frame_buffer()->ToI420();
  if (!frame_buffer) {
    RTC_LOG(LS_ERROR) << "Failed to convert "
                      << VideoFrameBufferTypeToString(
                             input_frame.video_frame_buffer()->type())
                      << " image to I420. Can't encode frame.";
    return WEBRTC_VIDEO_CODEC_ENCODER_FAILURE;
  }
  RTC_CHECK(frame_buffer->type() == VideoFrameBuffer::Type::kI420 ||
            frame_buffer->type() == VideoFrameBuffer::Type::kI420A);

  bool is_keyframe_needed = false;
  for (size_t i = 0; i < configurations_.size(); ++i) {
    if (configurations_[i].key_frame_request && configurations_[i].sending) {
      // 従来の挙動。初めて有効になった、または無効化後に再び有効になったレイヤーで
      // キーフレームを出すときは、全レイヤーでキーフレームを出す。
      is_keyframe_needed = true;
      break;
    }
  }

  RTC_DCHECK_EQ(configurations_[0].width, frame_buffer->width());
  RTC_DCHECK_EQ(configurations_[0].height, frame_buffer->height());

  int num_layers_to_send = 0;
  std::vector<VideoFrameType> frame_types_to_send(
      configurations_.size(), VideoFrameType::kVideoFrameDelta);
  for (size_t i = 0; i < encoders_.size(); ++i) {
    if (!configurations_[i].sending) {
      frame_types_to_send[i] = VideoFrameType::kEmptyFrame;
      continue;
    }

    const size_t simulcast_idx =
        static_cast<size_t>(configurations_[i].simulcast_idx);
    if (frame_types != nullptr && simulcast_idx < frame_types->size()) {
      frame_types_to_send[i] = (*frame_types)[simulcast_idx];
    }
    if (frame_types_to_send[i] != VideoFrameType::kEmptyFrame) {
      ++num_layers_to_send;
    }
  }

  // レイヤーごとに画像をエンコードする。
  for (size_t i = 0; i < encoders_.size(); ++i) {
    // EncodeFrame の入力。
    pictures_[i] = {0};
    pictures_[i].iPicWidth = configurations_[i].width;
    pictures_[i].iPicHeight = configurations_[i].height;
    pictures_[i].iColorFormat = EVideoFormatType::videoFormatI420;
    pictures_[i].uiTimeStamp = input_frame.ntp_time_ms();
    // 2 番目以降のレイヤーでは画像を縮小する。
    if (i == 0) {
      pictures_[i].iStride[0] = frame_buffer->StrideY();
      pictures_[i].iStride[1] = frame_buffer->StrideU();
      pictures_[i].iStride[2] = frame_buffer->StrideV();
      pictures_[i].pData[0] = const_cast<uint8_t*>(frame_buffer->DataY());
      pictures_[i].pData[1] = const_cast<uint8_t*>(frame_buffer->DataU());
      pictures_[i].pData[2] = const_cast<uint8_t*>(frame_buffer->DataV());
    } else {
      pictures_[i].iStride[0] = downscaled_buffers_[i - 1]->StrideY();
      pictures_[i].iStride[1] = downscaled_buffers_[i - 1]->StrideU();
      pictures_[i].iStride[2] = downscaled_buffers_[i - 1]->StrideV();
      pictures_[i].pData[0] =
          const_cast<uint8_t*>(downscaled_buffers_[i - 1]->DataY());
      pictures_[i].pData[1] =
          const_cast<uint8_t*>(downscaled_buffers_[i - 1]->DataU());
      pictures_[i].pData[2] =
          const_cast<uint8_t*>(downscaled_buffers_[i - 1]->DataV());
      // ダウンサンプリング係数に従って画像を縮小する。
      libyuv::I420Scale(pictures_[i - 1].pData[0], pictures_[i - 1].iStride[0],
                        pictures_[i - 1].pData[1], pictures_[i - 1].iStride[1],
                        pictures_[i - 1].pData[2], pictures_[i - 1].iStride[2],
                        configurations_[i - 1].width,
                        configurations_[i - 1].height, pictures_[i].pData[0],
                        pictures_[i].iStride[0], pictures_[i].pData[1],
                        pictures_[i].iStride[1], pictures_[i].pData[2],
                        pictures_[i].iStride[2], configurations_[i].width,
                        configurations_[i].height, libyuv::kFilterBox);
    }

    if (frame_types_to_send[i] == VideoFrameType::kEmptyFrame) {
      continue;
    }
    // このレイヤーがキーフレームを要求する設定か、明示的に要求されたときに送る。
    bool send_key_frame =
        is_keyframe_needed ||
        frame_types_to_send[i] == VideoFrameType::kVideoFrameKey;
    if (send_key_frame) {
      // API 文書では ForceIntraFrame(false) は何もしないとあるが、
      // 実際は `bIDR` の値に関係なくキーフレームを強制する。
      // (全フレームがキーフレームだと遅延が増える。)
      encoders_[i]->ForceIntraFrame(true);
      configurations_[i].key_frame_request = false;
    }
    // EncodeFrame の出力。
    SFrameBSInfo info;
    memset(&info, 0, sizeof(SFrameBSInfo));

    std::vector<ScalableVideoController::LayerFrameConfig> layer_frames;
    if (svc_controllers_[i]) {
      layer_frames = svc_controllers_[i]->NextFrameConfig(send_key_frame);
      RTC_CHECK_EQ(layer_frames.size(), 1);
    }

    // エンコードする。
    int enc_ret = encoders_[i]->EncodeFrame(&pictures_[i], &info);
    if (enc_ret != 0) {
      RTC_LOG(LS_ERROR)
          << "OpenH264 frame encoding failed, EncodeFrame returned " << enc_ret
          << ".";
      ReportError();
      return WEBRTC_VIDEO_CODEC_ERROR;
    }

    encoded_images_[i]._encodedWidth = configurations_[i].width;
    encoded_images_[i]._encodedHeight = configurations_[i].height;
    encoded_images_[i].SetRtpTimestamp(input_frame.rtp_timestamp());
    encoded_images_[i].SetColorSpace(input_frame.color_space());
    encoded_images_[i]._frameType = ConvertToVideoFrameType(info.eFrameType);
    encoded_images_[i].SetSimulcastIndex(configurations_[i].simulcast_idx);
    --num_layers_to_send;
    encoded_images_[i].set_end_of_temporal_unit(num_layers_to_send == 0);

    // エンコード画像をフラグメントに分割する。`encoded_image_` も更新する。
    RtpFragmentize(&encoded_images_[i], &info);

    // 帯域節約のためフレームをスキップすることがあり、そのときは
    // `encoded_images_[i]._length` == 0 になる。
    if (encoded_images_[i].size() > 0) {
      // QP を解析する。
      h264_bitstream_parser_.ParseBitstream(encoded_images_[i]);
      encoded_images_[i].qp_ =
          h264_bitstream_parser_.GetLastSliceQp().value_or(-1);

      // エンコード画像を届ける。
      CodecSpecificInfo codec_specific;
      codec_specific.codecType = kVideoCodecH264;
      codec_specific.codecSpecific.H264.packetization_mode =
          packetization_mode_;
      codec_specific.codecSpecific.H264.temporal_idx = kNoTemporalIdx;
      codec_specific.codecSpecific.H264.idr_frame =
          info.eFrameType == videoFrameTypeIDR;
      codec_specific.codecSpecific.H264.base_layer_sync = false;
      if (configurations_[i].num_temporal_layers > 1) {
        const uint8_t tid = info.sLayerInfo[0].uiTemporalId;
        codec_specific.codecSpecific.H264.temporal_idx = tid;
        codec_specific.codecSpecific.H264.base_layer_sync =
            tid > 0 && tid < tl0sync_limit_[i];
        if (svc_controllers_[i]) {
          if (encoded_images_[i]._frameType == VideoFrameType::kVideoFrameKey) {
            // キーフレームで ScalableVideoController をリセットし、
            // 想定する依存構造を戻す。
            layer_frames =
                svc_controllers_[i]->NextFrameConfig(/* restart= */ true);
            RTC_CHECK_EQ(layer_frames.size(), 1);
            RTC_DCHECK_EQ(layer_frames[0].TemporalId(), 0);
            RTC_DCHECK_EQ(layer_frames[0].IsKeyframe(), true);
          }

          if (layer_frames[0].TemporalId() != tid) {
            RTC_LOG(LS_WARNING)
                << "Encoder produced a frame with temporal id " << tid
                << ", expected " << layer_frames[0].TemporalId() << ".";
            continue;
          }
          encoded_images_[i].SetTemporalIndex(tid);
        }
        if (codec_specific.codecSpecific.H264.base_layer_sync) {
          tl0sync_limit_[i] = tid;
        }
        if (tid == 0) {
          tl0sync_limit_[i] = configurations_[i].num_temporal_layers;
        }
      }
      if (svc_controllers_[i]) {
        codec_specific.generic_frame_info =
            svc_controllers_[i]->OnEncodeDone(layer_frames[0]);
        if (send_key_frame && codec_specific.generic_frame_info.has_value()) {
          codec_specific.template_structure =
              svc_controllers_[i]->DependencyStructure();
        }
        codec_specific.scalability_mode = scalability_modes_[i];
      }
      encoded_image_callback_->OnEncodedImage(encoded_images_[i],
                                              &codec_specific);
    } else {
      encoded_image_callback_->OnFrameDropped(
          encoded_images_[i].RtpTimestamp(),
          *encoded_images_[i].SimulcastIndex(),
          *encoded_images_[i].is_end_of_temporal_unit());
    }
  }
  RTC_DCHECK_EQ(num_layers_to_send, 0);
  return WEBRTC_VIDEO_CODEC_OK;
}

// 初期化パラメータ。
// 初期化方法は 2 通りある。Initialize では memset でクリアした SEncParamBase を使う。
// InitializeExt では GetDefaultParams でクリアした SEncParamExt を使う。
// SEncParamExt は SEncParamBase の上位集合である。
SEncParamExt OpenH264VideoEncoder::CreateEncoderParams(size_t i) const {
  SEncParamExt encoder_params;
  encoders_[i]->GetDefaultParams(&encoder_params);
  if (codec_.mode == VideoCodecMode::kRealtimeVideo) {
    encoder_params.iUsageType = CAMERA_VIDEO_REAL_TIME;
  } else if (codec_.mode == VideoCodecMode::kScreensharing) {
    encoder_params.iUsageType = SCREEN_CONTENT_REAL_TIME;
  } else {
    RTC_DCHECK_NOTREACHED();
  }
  encoder_params.iPicWidth = configurations_[i].width;
  encoder_params.iPicHeight = configurations_[i].height;
  encoder_params.iTargetBitrate = configurations_[i].target_bps;
  // 未指定のままにする。WebRTC の最大コーデックビットレートは
  // OpenH264 の iMaxBitrate と同じ設定ではない。詳細は https://crbug.com/webrtc/11543
  encoder_params.iMaxBitrate = UNSPECIFIED_BIT_RATE;
  // レート制御モード
  encoder_params.iRCMode = RC_BITRATE_MODE;
  encoder_params.fMaxFrameRate = configurations_[i].max_frame_rate;

  // 以下は拡張パラメータ (SEncParamExt にあり SEncParamBase には無い)。
  encoder_params.bEnableFrameSkip = configurations_[i].frame_dropping_on;
  // `uiIntraPeriod`    - GOP サイズの倍数
  // `keyFrameInterval` - フレーム数
  encoder_params.uiIntraPeriod = configurations_[i].key_frame_interval;
  // 可能なら SPS ID を再利用する。Chromium の HW デコーダが
  // キーフレームごとにリセットされるのを避ける。
  // WebRTC は解像度変更でエンコーダをリセットするため、
  // INCREASING_ID (既定) 以外の EParameterSetStrategy は実質 CONSTANT_ID と同じになる。
  encoder_params.eSpsPpsIdStrategy = SPS_LISTING;
  encoder_params.uiMaxNalSize = 0;
  // スレッドモデル: auto を使う。
  //  0: auto (エンコーダ内部の動的実装)
  //  1: 単一スレッド (既定値)
  // >1: スレッド数
  encoder_params.iMultipleThreadIdc =
      NumberOfThreads(encoder_thread_limit_, encoder_params.iPicWidth,
                      encoder_params.iPicHeight, number_of_cores_);
  // 使う空間レイヤーはベースの 0 だけである。
  encoder_params.sSpatialLayers[0].iVideoWidth = encoder_params.iPicWidth;
  encoder_params.sSpatialLayers[0].iVideoHeight = encoder_params.iPicHeight;
  encoder_params.sSpatialLayers[0].fFrameRate = encoder_params.fMaxFrameRate;
  encoder_params.sSpatialLayers[0].iSpatialBitrate =
      encoder_params.iTargetBitrate;
  encoder_params.sSpatialLayers[0].iMaxSpatialBitrate =
      encoder_params.iMaxBitrate;
  encoder_params.iTemporalLayerNum = configurations_[i].num_temporal_layers;
  if (encoder_params.iTemporalLayerNum > 1) {
    // iNumRefFrame は確保する参照バッファの総数である。
    // 時間レイヤーが N 枚なら、参照時間レイヤーの直近フレームを置くため
    // 少なくとも (N - 1) 本のバッファが必要である。
    // OpenH264 エンコーダには、あるフレームの予測に使う参照集合を指定する API が無い。
    // エンコーダは理論上、利用可能な参照バッファをすべて使える。
    encoder_params.iNumRefFrame = encoder_params.iTemporalLayerNum - 1;
  }
  RTC_LOG(LS_INFO) << "OpenH264 version is " << OPENH264_MAJOR << "."
                   << OPENH264_MINOR;
  switch (packetization_mode_) {
    case H264PacketizationMode::SingleNalUnit:
      // 生成するパケットサイズを制限する。
      encoder_params.sSpatialLayers[0].sSliceArgument.uiSliceNum = 1;
      encoder_params.sSpatialLayers[0].sSliceArgument.uiSliceMode =
          SM_SIZELIMITED_SLICE;
      encoder_params.sSpatialLayers[0].sSliceArgument.uiSliceSizeConstraint =
          static_cast<unsigned int>(max_payload_size_);
      RTC_LOG(LS_INFO) << "Encoder is configured with NALU constraint: "
                       << max_payload_size_ << " bytes";
      break;
    case H264PacketizationMode::NonInterleaved:
      // uiSliceMode = SM_FIXEDSLCNUM_SLICE のとき uiSliceNum = 0 は
      // CPU コア数で自動設計することを意味する。
      // TODO(sprang): uiSliceNum > 1 でレート制御が壊れる理由が分かったら 0 にする。
      encoder_params.sSpatialLayers[0].sSliceArgument.uiSliceNum = 1;
      encoder_params.sSpatialLayers[0].sSliceArgument.uiSliceMode =
          SM_FIXEDSLCNUM_SLICE;
      break;
  }
  return encoder_params;
}

void OpenH264VideoEncoder::ReportInit() {
  if (has_reported_init_)
    return;
  RTC_HISTOGRAM_ENUMERATION("WebRTC.Video.OpenH264VideoEncoder.Event",
                            kH264EncoderEventInit, kH264EncoderEventMax);
  has_reported_init_ = true;
}

void OpenH264VideoEncoder::ReportError() {
  if (has_reported_error_)
    return;
  RTC_HISTOGRAM_ENUMERATION("WebRTC.Video.OpenH264VideoEncoder.Event",
                            kH264EncoderEventError, kH264EncoderEventMax);
  has_reported_error_ = true;
}

VideoEncoder::EncoderInfo OpenH264VideoEncoder::GetEncoderInfo() const {
  EncoderInfo info;
  info.supports_native_handle = false;
  info.implementation_name = "OpenH264";
  info.scaling_settings =
      VideoEncoder::ScalingSettings(kLowH264QpThreshold, kHighH264QpThreshold);
  info.is_hardware_accelerated = false;
  info.supports_simulcast = true;
  info.preferred_pixel_formats = {VideoFrameBuffer::Type::kI420};
  return info;
}

void OpenH264VideoEncoder::LayerConfig::SetStreamState(bool send_stream) {
  if (send_stream && !sending) {
    // このストリームをまだ送っていなければキーフレームが必要である。
    key_frame_request = true;
  }
  sending = send_stream;
}

}  // namespace webrtc

namespace sora {

std::unique_ptr<webrtc::VideoEncoder> CreateOpenH264VideoEncoder(
    const webrtc::SdpVideoFormat& format,
    std::string openh264) {
  webrtc::H264EncoderSettings settings;
  if (auto it = format.parameters.find(webrtc::kH264FmtpPacketizationMode);
      it != format.parameters.end()) {
    if (it->second == "0") {
      settings.packetization_mode =
          webrtc::H264PacketizationMode::SingleNalUnit;
    } else if (it->second == "1") {
      settings.packetization_mode =
          webrtc::H264PacketizationMode::NonInterleaved;
    }
  }

  return absl::make_unique<webrtc::OpenH264VideoEncoder>(
      webrtc::CreateEnvironment(), settings, std::move(openh264));
}

}  // namespace sora
