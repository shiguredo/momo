#include "sora/fix_cuda_noinline_macro_error.h"

#include "sora/hwenc_nvcodec/nvcodec_video_encoder.h"

#ifdef _WIN32
#include <d3d11.h>
#include <wrl.h>
#endif

#include <chrono>
#include <memory>
#include <mutex>
#include <queue>

// WebRTC
#include <api/video/nv12_buffer.h>
#include <api/video/video_frame.h>
#include <common_video/h264/h264_bitstream_parser.h>
#include <common_video/h265/h265_bitstream_parser.h>
#include <common_video/include/bitrate_adjuster.h>
#include <modules/video_coding/codecs/h264/include/h264.h>
#include <modules/video_coding/include/video_codec_interface.h>
#include <modules/video_coding/include/video_error_codes.h>
#include <rtc_base/logging.h>

// libyuv
#include <libyuv.h>

// NvCodec
#ifdef _WIN32
#include <NvEncoder/NvEncoderD3D11.h>
#endif
#ifdef __linux__
#include "nvcodec_video_encoder_cuda.h"
#endif

#ifdef __linux__
#include "sora/dyn/cuda.h"
#include "sora/dyn/nvcuvid.h"
#endif

namespace sora {

const int kLowH264QpThreshold = 34;
const int kHighH264QpThreshold = 40;

struct nal_entry {
  size_t offset;
  size_t size;
};

#ifdef _WIN32
using Microsoft::WRL::ComPtr;
#endif

class NvCodecVideoEncoderImpl : public NvCodecVideoEncoder {
 public:
  NvCodecVideoEncoderImpl(std::shared_ptr<CudaContext> cuda_context,
                          CudaVideoCodec codec);
  ~NvCodecVideoEncoderImpl() override;

  static bool IsSupported(std::shared_ptr<CudaContext> cuda_context,
                          CudaVideoCodec codec);

  int32_t InitEncode(const webrtc::VideoCodec* codec_settings,
                     int32_t number_of_cores,
                     size_t max_payload_size) override;
  int32_t RegisterEncodeCompleteCallback(
      webrtc::EncodedImageCallback* callback) override;
  int32_t Release() override;
  int32_t Encode(
      const webrtc::VideoFrame& frame,
      const std::vector<webrtc::VideoFrameType>* frame_types) override;
  void SetRates(
      const webrtc::VideoEncoder::RateControlParameters& parameters) override;
  webrtc::VideoEncoder::EncoderInfo GetEncoderInfo() const override;

  static std::unique_ptr<NvEncoder> CreateEncoder(
      CudaVideoCodec codec,
      int width,
      int height,
      int framerate,
      int target_bitrate_bps,
      int max_bitrate_bps
#ifdef _WIN32
      ,
      ID3D11Device* id3d11_device,
      ID3D11Texture2D** out_id3d11_texture
#endif
#ifdef __linux__
      ,
      NvCodecVideoEncoderCuda* cuda,
      bool is_nv12
#endif
  );

 private:
  std::mutex mutex_;
  webrtc::EncodedImageCallback* callback_ = nullptr;
  webrtc::BitrateAdjuster bitrate_adjuster_;
  uint32_t target_bitrate_bps_ = 0;
  uint32_t max_bitrate_bps_ = 0;

  int32_t InitNvEnc();
  int32_t ReleaseNvEnc();
  webrtc::H264BitstreamParser h264_bitstream_parser_;
  webrtc::H265BitstreamParser h265_bitstream_parser_;

  std::shared_ptr<CudaContext> cuda_context_;
  CudaVideoCodec codec_;
  std::unique_ptr<NvEncoder> nv_encoder_;
#ifdef _WIN32
  Microsoft::WRL::ComPtr<ID3D11Device> id3d11_device_;
  Microsoft::WRL::ComPtr<ID3D11DeviceContext> id3d11_context_;
  Microsoft::WRL::ComPtr<ID3D11Texture2D> id3d11_texture_;
#endif
#ifdef __linux__
  std::unique_ptr<NvCodecVideoEncoderCuda> cuda_;
  bool is_nv12_ = false;
#endif
  bool reconfigure_needed_ = false;
  uint32_t width_ = 0;
  uint32_t height_ = 0;
  uint32_t framerate_ = 0;
  webrtc::VideoCodecMode mode_ = webrtc::VideoCodecMode::kRealtimeVideo;
  NV_ENC_INITIALIZE_PARAMS initialize_params_;
  std::vector<std::vector<uint8_t>> v_packet_;
  webrtc::EncodedImage encoded_image_;
};

NvCodecVideoEncoderImpl::NvCodecVideoEncoderImpl(
    std::shared_ptr<CudaContext> cuda_context,
    CudaVideoCodec codec)
    : cuda_context_(cuda_context), codec_(codec), bitrate_adjuster_(0.5, 0.95) {
#ifdef _WIN32
  ComPtr<IDXGIFactory1> idxgi_factory;
  RTC_CHECK(!FAILED(CreateDXGIFactory1(__uuidof(IDXGIFactory1),
                                       (void**)idxgi_factory.GetAddressOf())));
  ComPtr<IDXGIAdapter> idxgi_adapter;
  RTC_CHECK(
      !FAILED(idxgi_factory->EnumAdapters(0, idxgi_adapter.GetAddressOf())));
  RTC_CHECK(!FAILED(D3D11CreateDevice(
      idxgi_adapter.Get(), D3D_DRIVER_TYPE_UNKNOWN, NULL, 0, NULL, 0,
      D3D11_SDK_VERSION, id3d11_device_.GetAddressOf(), NULL,
      id3d11_context_.GetAddressOf())));

  // 以下デバイス名を取得するだけの処理
  DXGI_ADAPTER_DESC adapter_desc;
  idxgi_adapter->GetDesc(&adapter_desc);
  char szDesc[80];
  size_t result = 0;
  wcstombs_s(&result, szDesc, adapter_desc.Description, sizeof(szDesc));
  RTC_LOG(LS_INFO) << __FUNCTION__ << "GPU in use: " << szDesc;
#endif
#ifdef __linux__
  cuda_.reset(new NvCodecVideoEncoderCuda(cuda_context_));
#endif
}

NvCodecVideoEncoderImpl::~NvCodecVideoEncoderImpl() {}

int32_t NvCodecVideoEncoderImpl::InitEncode(
    const webrtc::VideoCodec* codec_settings,
    int32_t number_of_cores,
    size_t max_payload_size) {
  RTC_DCHECK(codec_settings);

  int32_t release_ret = Release();
  if (release_ret != WEBRTC_VIDEO_CODEC_OK) {
    return release_ret;
  }

  width_ = codec_settings->width;
  height_ = codec_settings->height;
  target_bitrate_bps_ = codec_settings->startBitrate * 1000;
  max_bitrate_bps_ = codec_settings->maxBitrate * 1000;
  bitrate_adjuster_.SetTargetBitrateBps(target_bitrate_bps_);
  framerate_ = codec_settings->maxFramerate;
  mode_ = codec_settings->mode;

  RTC_LOG(LS_INFO) << "InitEncode " << target_bitrate_bps_ << "bit/sec";

  return InitNvEnc();
}

int32_t NvCodecVideoEncoderImpl::RegisterEncodeCompleteCallback(
    webrtc::EncodedImageCallback* callback) {
  std::lock_guard<std::mutex> lock(mutex_);
  callback_ = callback;
  return WEBRTC_VIDEO_CODEC_OK;
}

int32_t NvCodecVideoEncoderImpl::Release() {
  return ReleaseNvEnc();
}

int32_t NvCodecVideoEncoderImpl::Encode(
    const webrtc::VideoFrame& frame,
    const std::vector<webrtc::VideoFrameType>* frame_types) {
  //RTC_LOG(LS_ERROR) << __FUNCTION__ << " Start";
  if (!nv_encoder_) {
    return WEBRTC_VIDEO_CODEC_UNINITIALIZED;
  }
  if (!callback_) {
    RTC_LOG(LS_WARNING)
        << "InitEncode() has been called, but a callback function "
        << "has not been set with RegisterEncodeCompleteCallback()";
    return WEBRTC_VIDEO_CODEC_UNINITIALIZED;
  }

#if defined(__linux__)
  if (frame.video_frame_buffer()->type() ==
      webrtc::VideoFrameBuffer::Type::kNV12) {
    if (!is_nv12_) {
      ReleaseNvEnc();
      is_nv12_ = true;
      InitNvEnc();
    }
  } else {
    if (is_nv12_) {
      ReleaseNvEnc();
      is_nv12_ = false;
      InitNvEnc();
    }
  }
#endif

  bool send_key_frame = false;

  if (reconfigure_needed_) {
    NV_ENC_RECONFIGURE_PARAMS reconfigure_params = {
        NV_ENC_RECONFIGURE_PARAMS_VER};
    NV_ENC_CONFIG encode_config = {NV_ENC_CONFIG_VER};
    reconfigure_params.reInitEncodeParams.encodeConfig = &encode_config;
    nv_encoder_->GetInitializeParams(&reconfigure_params.reInitEncodeParams);

    reconfigure_params.reInitEncodeParams.frameRateNum = framerate_;

    encode_config.rcParams.averageBitRate =
        bitrate_adjuster_.GetAdjustedBitrateBps();
    encode_config.rcParams.maxBitRate = max_bitrate_bps_;
    encode_config.rcParams.vbvBufferSize =
        encode_config.rcParams.averageBitRate * 1 / framerate_;
    encode_config.rcParams.vbvInitialDelay =
        encode_config.rcParams.vbvBufferSize;
    try {
      //RTC_LOG(LS_ERROR) << __FUNCTION__ << " Reconfigure";
      nv_encoder_->Reconfigure(&reconfigure_params);
    } catch (const NVENCException& e) {
      RTC_LOG(LS_ERROR) << __FUNCTION__ << e.what();
      return WEBRTC_VIDEO_CODEC_ERROR;
    }

    reconfigure_needed_ = false;
  }

  if (frame_types != nullptr) {
    // We only support a single stream.
    RTC_DCHECK_EQ(frame_types->size(), static_cast<size_t>(1));
    // Skip frame?
    if ((*frame_types)[0] == webrtc::VideoFrameType::kEmptyFrame) {
      return WEBRTC_VIDEO_CODEC_OK;
    }
    // Force key frame?
    send_key_frame =
        (*frame_types)[0] == webrtc::VideoFrameType::kVideoFrameKey;
  }

  NV_ENC_PIC_PARAMS pic_params = {NV_ENC_PIC_PARAMS_VER};
  pic_params.encodePicFlags = 0;
  if (send_key_frame) {
    pic_params.encodePicFlags =
        NV_ENC_PIC_FLAG_FORCEINTRA | NV_ENC_PIC_FLAG_FORCEIDR;
  }
  pic_params.inputWidth = width_;
  pic_params.inputHeight = height_;

  v_packet_.clear();

#ifdef _WIN32
  const NvEncInputFrame* input_frame = nv_encoder_->GetNextInputFrame();
  D3D11_MAPPED_SUBRESOURCE map;
  id3d11_context_->Map(id3d11_texture_.Get(), D3D11CalcSubresource(0, 0, 1),
                       D3D11_MAP_WRITE, 0, &map);
  if (frame.video_frame_buffer()->type() ==
      webrtc::VideoFrameBuffer::Type::kNV12) {
    webrtc::NV12BufferInterface* frame_buffer =
        static_cast<webrtc::NV12BufferInterface*>(
            frame.video_frame_buffer().get());
    libyuv::NV12Copy(
        frame_buffer->DataY(), frame_buffer->StrideY(), frame_buffer->DataUV(),
        frame_buffer->StrideUV(), (uint8_t*)map.pData, map.RowPitch,
        ((uint8_t*)map.pData + height_ * map.RowPitch), map.RowPitch,
        frame_buffer->width(), frame_buffer->height());
  } else {
    webrtc::scoped_refptr<const webrtc::I420BufferInterface> frame_buffer =
        frame.video_frame_buffer()->ToI420();
    libyuv::I420ToNV12(
        frame_buffer->DataY(), frame_buffer->StrideY(), frame_buffer->DataU(),
        frame_buffer->StrideU(), frame_buffer->DataV(), frame_buffer->StrideV(),
        (uint8_t*)map.pData, map.RowPitch,
        ((uint8_t*)map.pData + height_ * map.RowPitch), map.RowPitch,
        frame_buffer->width(), frame_buffer->height());
  }
  id3d11_context_->Unmap(id3d11_texture_.Get(), D3D11CalcSubresource(0, 0, 1));
  ID3D11Texture2D* nv11_texture =
      reinterpret_cast<ID3D11Texture2D*>(input_frame->inputPtr);
  id3d11_context_->CopyResource(nv11_texture, id3d11_texture_.Get());
#endif
#ifdef __linux__

  //RTC_LOG(LS_INFO) << "type="
  //                 << VideoFrameBufferTypeToString(
  //                        frame.video_frame_buffer()->type())
  //                 << " width_=" << width_ << " height_=" << height_
  //                 << " frame_width=" << frame.video_frame_buffer()->width()
  //                 << " frame_height=" << frame.video_frame_buffer()->height();

  if (frame.video_frame_buffer()->type() ==
      webrtc::VideoFrameBuffer::Type::kNV12) {
    webrtc::NV12Buffer* buffer =
        static_cast<webrtc::NV12Buffer*>(frame.video_frame_buffer().get());
    try {
      cuda_->Copy(nv_encoder_.get(), buffer->DataY(), width_, height_);
    } catch (const NVENCException& e) {
      RTC_LOG(LS_ERROR) << e.what();
      return WEBRTC_VIDEO_CODEC_ERROR;
    }
  } else {
    webrtc::scoped_refptr<const webrtc::I420BufferInterface> frame_buffer =
        frame.video_frame_buffer()->ToI420();
    cuda_->Copy(nv_encoder_.get(), frame_buffer->DataY(), frame_buffer->width(),
                frame_buffer->height());
  }
#endif

  try {
    nv_encoder_->EncodeFrame(v_packet_, &pic_params);
  } catch (const NVENCException& e) {
    RTC_LOG(LS_ERROR) << __FUNCTION__ << e.what();
    return WEBRTC_VIDEO_CODEC_ERROR;
  }

  for (std::vector<uint8_t>& packet : v_packet_) {
    auto encoded_image_buffer =
        webrtc::EncodedImageBuffer::Create(packet.data(), packet.size());
    encoded_image_.SetEncodedData(encoded_image_buffer);
    encoded_image_._encodedWidth = width_;
    encoded_image_._encodedHeight = height_;
    encoded_image_.content_type_ =
        (mode_ == webrtc::VideoCodecMode::kScreensharing)
            ? webrtc::VideoContentType::SCREENSHARE
            : webrtc::VideoContentType::UNSPECIFIED;
    encoded_image_.timing_.flags = webrtc::VideoSendTiming::kInvalid;
    encoded_image_.SetRtpTimestamp(frame.rtp_timestamp());
    encoded_image_.ntp_time_ms_ = frame.ntp_time_ms();
    encoded_image_.capture_time_ms_ = frame.render_time_ms();
    encoded_image_.rotation_ = frame.rotation();
    encoded_image_.SetColorSpace(frame.color_space());
    encoded_image_._frameType = webrtc::VideoFrameType::kVideoFrameDelta;

    uint8_t zero_count = 0;
    size_t nal_start_idx = 0;
    for (size_t i = 0; i < packet.size(); i++) {
      uint8_t data = packet.data()[i];
      if ((i != 0) && (i == nal_start_idx)) {
        if ((data & 0x1F) == 0x05) {
          encoded_image_._frameType = webrtc::VideoFrameType::kVideoFrameKey;
        }
      }
      if (data == 0x01 && zero_count >= 2) {
        nal_start_idx = i + 1;
      }
      if (data == 0x00) {
        zero_count++;
      } else {
        zero_count = 0;
      }
    }

    webrtc::CodecSpecificInfo codec_specific;
    if (codec_ == CudaVideoCodec::H264) {
      codec_specific.codecType = webrtc::kVideoCodecH264;
      codec_specific.codecSpecific.H264.packetization_mode =
          webrtc::H264PacketizationMode::NonInterleaved;

      h264_bitstream_parser_.ParseBitstream(encoded_image_);
      encoded_image_.qp_ = h264_bitstream_parser_.GetLastSliceQp().value_or(-1);
    } else if (codec_ == CudaVideoCodec::H265) {
      codec_specific.codecType = webrtc::kVideoCodecH265;

      h265_bitstream_parser_.ParseBitstream(encoded_image_);
      encoded_image_.qp_ = h265_bitstream_parser_.GetLastSliceQp().value_or(-1);
    }

    webrtc::EncodedImageCallback::Result result =
        callback_->OnEncodedImage(encoded_image_, &codec_specific);
    if (result.error != webrtc::EncodedImageCallback::Result::OK) {
      RTC_LOG(LS_ERROR) << __FUNCTION__
                        << " OnEncodedImage failed error:" << result.error;
      return WEBRTC_VIDEO_CODEC_ERROR;
    }
    bitrate_adjuster_.Update(packet.size());
  }

  return WEBRTC_VIDEO_CODEC_OK;
}

void NvCodecVideoEncoderImpl::SetRates(
    const webrtc::VideoEncoder::RateControlParameters& parameters) {
  if (!nv_encoder_) {
    RTC_LOG(LS_WARNING) << "SetRates() while uninitialized.";
    return;
  }

  if (parameters.framerate_fps < 1.0) {
    RTC_LOG(LS_WARNING) << "Invalid frame rate: " << parameters.framerate_fps;
    return;
  }

  uint32_t new_framerate = (uint32_t)parameters.framerate_fps;
  uint32_t new_bitrate = parameters.bitrate.get_sum_bps();
  RTC_LOG(LS_INFO) << __FUNCTION__ << " framerate_:" << framerate_
                   << " new_framerate: " << new_framerate
                   << " target_bitrate_bps_:" << target_bitrate_bps_
                   << " new_bitrate:" << new_bitrate
                   << " max_bitrate_bps_:" << max_bitrate_bps_;
  framerate_ = new_framerate;
  target_bitrate_bps_ = new_bitrate;
  bitrate_adjuster_.SetTargetBitrateBps(target_bitrate_bps_);
  reconfigure_needed_ = true;
}

webrtc::VideoEncoder::EncoderInfo NvCodecVideoEncoderImpl::GetEncoderInfo()
    const {
  webrtc::VideoEncoder::EncoderInfo info;
  info.supports_native_handle = true;
  info.implementation_name = "NvCodec";
  info.scaling_settings = webrtc::VideoEncoder::ScalingSettings(
      kLowH264QpThreshold, kHighH264QpThreshold);
  return info;
}

int32_t NvCodecVideoEncoderImpl::InitNvEnc() {
#ifdef _WIN32
  nv_encoder_ = CreateEncoder(
      codec_, width_, height_, framerate_, target_bitrate_bps_,
      max_bitrate_bps_, id3d11_device_.Get(), id3d11_texture_.GetAddressOf());
#endif
#ifdef __linux__
  nv_encoder_ =
      CreateEncoder(codec_, width_, height_, framerate_, target_bitrate_bps_,
                    max_bitrate_bps_, cuda_.get(), is_nv12_);
#endif

  if (nv_encoder_ == nullptr) {
    return WEBRTC_VIDEO_CODEC_ERROR;
  }

  reconfigure_needed_ = false;

  return WEBRTC_VIDEO_CODEC_OK;
}

int32_t NvCodecVideoEncoderImpl::ReleaseNvEnc() {
  if (nv_encoder_) {
    try {
      nv_encoder_->EndEncode(v_packet_);
      nv_encoder_->DestroyEncoder();
    } catch (const NVENCException& e) {
      RTC_LOG(LS_ERROR) << __FUNCTION__ << e.what();
    }
    nv_encoder_ = nullptr;
#ifdef _WIN32
    id3d11_texture_.Reset();
#endif
  }
  return WEBRTC_VIDEO_CODEC_OK;
}

std::unique_ptr<NvEncoder> NvCodecVideoEncoderImpl::CreateEncoder(
    CudaVideoCodec codec,
    int width,
    int height,
    int framerate,
    int target_bitrate_bps,
    int max_bitrate_bps
#ifdef _WIN32
    ,
    ID3D11Device* id3d11_device,
    ID3D11Texture2D** out_id3d11_texture
#endif
#ifdef __linux__
    ,
    NvCodecVideoEncoderCuda* cuda,
    bool is_nv12
#endif
) {
  std::unique_ptr<NvEncoder> encoder;

#ifdef _WIN32
  DXGI_FORMAT dxgi_format = DXGI_FORMAT_NV12;
  NV_ENC_BUFFER_FORMAT nvenc_format = NV_ENC_BUFFER_FORMAT_NV12;
  D3D11_TEXTURE2D_DESC desc;
  ZeroMemory(&desc, sizeof(D3D11_TEXTURE2D_DESC));
  desc.Width = width;
  desc.Height = height;
  desc.MipLevels = 1;
  desc.ArraySize = 1;
  desc.Format = dxgi_format;
  desc.SampleDesc.Count = 1;
  desc.Usage = D3D11_USAGE_STAGING;
  desc.BindFlags = 0;
  desc.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
  id3d11_device->CreateTexture2D(&desc, NULL, out_id3d11_texture);

  // Driver が古いとかに気づくのはココ
  try {
    encoder.reset(
        new NvEncoderD3D11(id3d11_device, width, height, nvenc_format));
  } catch (const NVENCException& e) {
    RTC_LOG(LS_ERROR) << __FUNCTION__ << e.what();
    return nullptr;
  }
#endif

#ifdef __linux__
  try {
    encoder.reset(cuda->CreateNvEncoder(width, height, is_nv12));
  } catch (const NVENCException& e) {
    RTC_LOG(LS_ERROR) << __FUNCTION__ << e.what();
    return nullptr;
  }
#endif

  NV_ENC_INITIALIZE_PARAMS initialize_params = {NV_ENC_INITIALIZE_PARAMS_VER};
  NV_ENC_CONFIG encode_config = {NV_ENC_CONFIG_VER};
  initialize_params.encodeConfig = &encode_config;
  try {
    if (codec == CudaVideoCodec::H264) {
      encoder->CreateDefaultEncoderParams(
          &initialize_params, NV_ENC_CODEC_H264_GUID, NV_ENC_PRESET_P3_GUID,
          NV_ENC_TUNING_INFO_LOW_LATENCY);
    } else if (codec == CudaVideoCodec::H265) {
      encoder->CreateDefaultEncoderParams(
          &initialize_params, NV_ENC_CODEC_HEVC_GUID, NV_ENC_PRESET_P2_GUID,
          NV_ENC_TUNING_INFO_LOW_LATENCY);
    }

    //initialize_params.enablePTD = 1;
    initialize_params.frameRateDen = 1;
    initialize_params.frameRateNum = framerate;
    initialize_params.maxEncodeWidth = width;
    initialize_params.maxEncodeHeight = height;

    //encode_config.profileGUID = NV_ENC_H264_PROFILE_BASELINE_GUID;
    //encode_config.rcParams.rateControlMode = NV_ENC_PARAMS_RC_CBR_LOWDELAY_HQ;
    encode_config.rcParams.averageBitRate = target_bitrate_bps;
    encode_config.rcParams.maxBitRate = max_bitrate_bps;

    encode_config.rcParams.disableBadapt = 1;
    encode_config.rcParams.vbvBufferSize =
        encode_config.rcParams.averageBitRate * initialize_params.frameRateDen /
        initialize_params.frameRateNum;
    encode_config.rcParams.vbvInitialDelay =
        encode_config.rcParams.vbvBufferSize;
    encode_config.gopLength = NVENC_INFINITE_GOPLENGTH;
    encode_config.frameIntervalP = 1;
    encode_config.rcParams.enableAQ = 1;

    if (codec == CudaVideoCodec::H264) {
      //encode_config.encodeCodecConfig.h264Config.outputAUD = 1;
      //encode_config.encodeCodecConfig.h264Config.level = NV_ENC_LEVEL_H264_31;
      //encode_config.encodeCodecConfig.h264Config.entropyCodingMode = NV_ENC_H264_ENTROPY_CODING_MODE_CAVLC;
      encode_config.encodeCodecConfig.h264Config.idrPeriod =
          NVENC_INFINITE_GOPLENGTH;
      encode_config.encodeCodecConfig.h264Config.repeatSPSPPS = 1;
      encode_config.encodeCodecConfig.h264Config.sliceMode = 0;
      encode_config.encodeCodecConfig.h264Config.sliceModeData = 0;
    } else if (codec == CudaVideoCodec::H265) {
      encode_config.encodeCodecConfig.hevcConfig.idrPeriod =
          NVENC_INFINITE_GOPLENGTH;
      encode_config.encodeCodecConfig.hevcConfig.repeatSPSPPS = 1;
      encode_config.encodeCodecConfig.hevcConfig.sliceMode = 0;
      encode_config.encodeCodecConfig.hevcConfig.sliceModeData = 0;
    }

    encoder->CreateEncoder(&initialize_params);

    RTC_LOG(LS_INFO) << __FUNCTION__ << " framerate:" << framerate
                     << " bitrate_bps:" << target_bitrate_bps
                     << " maxBitRate:" << encode_config.rcParams.maxBitRate;
  } catch (const NVENCException& e) {
    RTC_LOG(LS_ERROR) << __FUNCTION__ << ": " << e.what();
    return nullptr;
  }

  return encoder;
}

bool NvCodecVideoEncoder::IsSupported(std::shared_ptr<CudaContext> cuda_context,
                                      CudaVideoCodec codec) {
  try {
    NvEncoder::TryLoadNvEncApi();

    // Linux の場合、cuda と nvcuvid のロードも必要なのでチェックする
#ifdef __linux__
    if (cuda_context == nullptr) {
      return false;
    }

    if (!dyn::DynModule::Instance().IsLoadable(dyn::CUDA_SO)) {
      return false;
    }
    if (!dyn::DynModule::Instance().IsLoadable(dyn::NVCUVID_SO)) {
      return false;
    }
    // 関数が存在するかチェックする
    if (dyn::DynModule::Instance().GetFunc(dyn::CUDA_SO, "cuDeviceGetName") ==
        nullptr) {
      return false;
    }
    if (dyn::DynModule::Instance().GetFunc(dyn::NVCUVID_SO,
                                           "cuvidMapVideoFrame") == nullptr) {
      return false;
    }
#endif

    // 実際にエンコーダを作れるかを確認する
#ifdef _WIN32
    ComPtr<IDXGIFactory1> idxgi_factory;
    RTC_CHECK(!FAILED(CreateDXGIFactory1(
        __uuidof(IDXGIFactory1), (void**)idxgi_factory.GetAddressOf())));
    ComPtr<IDXGIAdapter> idxgi_adapter;
    RTC_CHECK(
        !FAILED(idxgi_factory->EnumAdapters(0, idxgi_adapter.GetAddressOf())));
    Microsoft::WRL::ComPtr<ID3D11Device> id3d11_device;
    Microsoft::WRL::ComPtr<ID3D11DeviceContext> id3d11_context;
    Microsoft::WRL::ComPtr<ID3D11Texture2D> id3d11_texture;
    RTC_CHECK(!FAILED(D3D11CreateDevice(
        idxgi_adapter.Get(), D3D_DRIVER_TYPE_UNKNOWN, NULL, 0, NULL, 0,
        D3D11_SDK_VERSION, id3d11_device.GetAddressOf(), NULL,
        id3d11_context.GetAddressOf())));

    auto encoder = NvCodecVideoEncoderImpl::CreateEncoder(
        codec, 640, 480, 30, 100 * 1000, 500 * 1000, id3d11_device.Get(),
        id3d11_texture.GetAddressOf());
#endif
#ifdef __linux__
    auto cuda = std::unique_ptr<NvCodecVideoEncoderCuda>(
        new NvCodecVideoEncoderCuda(cuda_context));
    auto encoder = NvCodecVideoEncoderImpl::CreateEncoder(
        codec, 640, 480, 30, 100 * 1000, 500 * 1000, cuda.get(), true);
#endif
    if (encoder == nullptr) {
      return false;
    }

    return true;
  } catch (const NVENCException& e) {
    RTC_LOG(LS_ERROR) << __FUNCTION__ << ": " << e.what();
    return false;
  }
}

std::unique_ptr<NvCodecVideoEncoder> NvCodecVideoEncoder::Create(
    std::shared_ptr<CudaContext> cuda_context,
    CudaVideoCodec codec) {
  return std::unique_ptr<NvCodecVideoEncoder>(
      new NvCodecVideoEncoderImpl(cuda_context, codec));
}

}  // namespace sora
