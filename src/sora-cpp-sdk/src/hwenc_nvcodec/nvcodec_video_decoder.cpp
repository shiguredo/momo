#include "sora/hwenc_nvcodec/nvcodec_video_decoder.h"

// WebRTC
#include <modules/video_coding/include/video_error_codes.h>
#include <rtc_base/checks.h>
#include <rtc_base/logging.h>
#include <rtc_base/time_utils.h>
#include <third_party/libyuv/include/libyuv/convert.h>

// NvCodec
#include <NvDecoder/NvDecoder.h>

#include "sora/dyn/cuda.h"
#include "sora/dyn/nvcuvid.h"

namespace sora {

NvCodecVideoDecoder::NvCodecVideoDecoder(std::shared_ptr<CudaContext> ctx,
                                         CudaVideoCodec codec)
    : context_(ctx),
      codec_(codec),
      decode_complete_callback_(nullptr),
      buffer_pool_(false, 300 /* max_number_of_buffers*/) {}

NvCodecVideoDecoder::~NvCodecVideoDecoder() {
  Release();
}

bool NvCodecVideoDecoder::IsSupported(std::shared_ptr<CudaContext> context,
                                      CudaVideoCodec codec) {
  if (context == nullptr) {
    return false;
  }

  // CUDA 周りのライブラリがロードできるか確認する
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

  try {
    auto p = new NvCodecDecoderCuda(context, codec);
    delete p;
    return true;
  } catch (...) {
    return false;
  }
}

bool NvCodecVideoDecoder::Configure(const Settings& settings) {
  return InitNvCodec();
}

int32_t NvCodecVideoDecoder::Decode(const webrtc::EncodedImage& input_image,
                                    bool missing_frames,
                                    int64_t render_time_ms) {
  if (decoder_ == nullptr) {
    return WEBRTC_VIDEO_CODEC_UNINITIALIZED;
  }
  if (decode_complete_callback_ == nullptr) {
    return WEBRTC_VIDEO_CODEC_UNINITIALIZED;
  }
  if (input_image.data() == nullptr && input_image.size() > 0) {
    return WEBRTC_VIDEO_CODEC_ERR_PARAMETER;
  }

  uint8_t** frames = nullptr;
  int frame_count;
  try {
    frame_count = decoder_->Decode(input_image.data(), (int)input_image.size());
  } catch (NVDECException& e) {
    RTC_LOG(LS_ERROR) << e.getErrorString();
    return WEBRTC_VIDEO_CODEC_ERROR;
  }
  if (frame_count == 0) {
    return WEBRTC_VIDEO_CODEC_OK;
  }

  // 初回だけデコード情報を出力する
  if (!output_info_) {
    RTC_LOG(LS_INFO) << decoder_->GetVideoInfo();
    output_info_ = true;
  }

  uint32_t pts = input_image.RtpTimestamp();

  for (int i = 0; i < frame_count; i++) {
    auto* frame = decoder_->GetLockedFrame();
    // NV12 から I420 に変換
    rtc::scoped_refptr<webrtc::I420Buffer> i420_buffer =
        buffer_pool_.CreateI420Buffer(decoder_->GetWidth(),
                                      decoder_->GetHeight());
    libyuv::NV12ToI420(
        frame, decoder_->GetDeviceFramePitch(),
        frame + decoder_->GetHeight() * decoder_->GetDeviceFramePitch(),
        decoder_->GetDeviceFramePitch(), i420_buffer->MutableDataY(),
        i420_buffer->StrideY(), i420_buffer->MutableDataU(),
        i420_buffer->StrideU(), i420_buffer->MutableDataV(),
        i420_buffer->StrideV(), decoder_->GetWidth(), decoder_->GetHeight());

    webrtc::VideoFrame decoded_image = webrtc::VideoFrame::Builder()
                                           .set_video_frame_buffer(i420_buffer)
                                           .set_timestamp_rtp(pts)
                                           .build();
    decode_complete_callback_->Decoded(decoded_image, absl::nullopt,
                                       absl::nullopt);

    decoder_->UnlockFrame(frame);
  }
  // 次のフレームで縦横サイズが変わったときに追従するためのマジックコード
  decoder_->setReconfigParams();

  return WEBRTC_VIDEO_CODEC_OK;
}

int32_t NvCodecVideoDecoder::RegisterDecodeCompleteCallback(
    webrtc::DecodedImageCallback* callback) {
  decode_complete_callback_ = callback;
  return WEBRTC_VIDEO_CODEC_OK;
}

int32_t NvCodecVideoDecoder::Release() {
  ReleaseNvCodec();
  buffer_pool_.Release();
  return WEBRTC_VIDEO_CODEC_OK;
}

const char* NvCodecVideoDecoder::ImplementationName() const {
  return "NVIDIA VIDEO CODEC SDK";
}

bool NvCodecVideoDecoder::InitNvCodec() {
  decoder_.reset(new NvCodecDecoderCuda(context_, codec_));
  output_info_ = false;
  return true;
}

void NvCodecVideoDecoder::ReleaseNvCodec() {
  decoder_.reset();
}

}  // namespace sora
