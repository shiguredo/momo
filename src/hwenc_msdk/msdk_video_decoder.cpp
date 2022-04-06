#include "msdk_video_decoder.h"

#include <thread>

// Linux
#include <unistd.h>

// WebRTC
#include <modules/video_coding/include/video_error_codes.h>
#include <rtc_base/checks.h>
#include <rtc_base/logging.h>
#include <rtc_base/time_utils.h>
#include <third_party/libyuv/include/libyuv/convert.h>

MsdkVideoDecoder::MsdkVideoDecoder(mfxU32 codec_id)
    : codec_id_(codec_id),
      decoder_(nullptr),
      decode_complete_callback_(nullptr),
      buffer_pool_(false, 300 /* max_number_of_buffers*/) {
  mfxStatus sts = MFX_ERR_NONE;

  mfxIMPL impl = MFX_IMPL_HARDWARE;
  mfxVersion ver = {{0, 1}};

  // Initialize Intel Media SDK Session
  sts = session_.Init(impl, &ver);
  MSDK_CHECK_RESULT(sts, MFX_ERR_NONE, sts);

  // Create VA display
  libva_ = CreateDRMLibVA();
  if (!libva_) {
    RTC_LOG(LS_ERROR) << "Failed to create DRM VA display";
    throw 1;
  }

  // Provide VA display handle to Media SDK
  sts = session_.SetHandle(static_cast<mfxHandleType>(MFX_HANDLE_VA_DISPLAY),
                           libva_->GetVADisplay());
  MSDK_CHECK_RESULT(sts, MFX_ERR_NONE, sts);

  // Query selected implementation and version
  sts = session_.QueryIMPL(&impl);
  MSDK_CHECK_RESULT(sts, MFX_ERR_NONE, sts);

  sts = session_.QueryVersion(&ver);
  MSDK_CHECK_RESULT(sts, MFX_ERR_NONE, sts);

  RTC_LOG(LS_INFO) << "Intel Media SDK Implementation: "
                   << (impl == MFX_IMPL_SOFTWARE ? "SOFTWARE" : "HARDWARE");
  RTC_LOG(LS_INFO) << "Intel Media SDK API Version: " << ver.Major << "."
                   << ver.Minor;
}

MsdkVideoDecoder::~MsdkVideoDecoder() {
  Release();
}

bool MsdkVideoDecoder::IsSupported(mfxU32 codec) {
  std::unique_ptr<DRMLibVA> libva = CreateDRMLibVA();
  if (!libva) {
    return false;
  }

  MFXVideoSession session;

  mfxStatus sts = MFX_ERR_NONE;

  sts = session.SetHandle(static_cast<mfxHandleType>(MFX_HANDLE_VA_DISPLAY),
                          libva->GetVADisplay());
  if (sts != MFX_ERR_NONE) {
    return false;
  }

  mfxIMPL impl = MFX_IMPL_HARDWARE;
  sts = session.QueryIMPL(&impl);
  if (sts != MFX_ERR_NONE) {
    return false;
  }

  mfxVersion ver = {{0, 1}};
  sts = session.QueryVersion(&ver);
  if (sts != MFX_ERR_NONE) {
    return false;
  }

  int width = 640;
  int height = 480;

  std::unique_ptr<MFXVideoDECODE> decoder;
  decoder.reset(new MFXVideoDECODE(session));

  mfxVideoParam param;
  memset(&param, 0, sizeof(param));

  param.mfx.CodecId = codec;
  param.mfx.FrameInfo.FourCC = MFX_FOURCC_NV12;
  param.mfx.FrameInfo.ChromaFormat = MFX_CHROMAFORMAT_YUV420;
  param.mfx.FrameInfo.PicStruct = MFX_PICSTRUCT_PROGRESSIVE;
  param.mfx.FrameInfo.CropX = 0;
  param.mfx.FrameInfo.CropY = 0;
  param.mfx.FrameInfo.CropW = width;
  param.mfx.FrameInfo.CropH = height;
  param.mfx.FrameInfo.Width = (width + 15) / 16 * 16;
  param.mfx.FrameInfo.Height = (height + 15) / 16 * 16;

  param.mfx.GopRefDist = 1;
  param.AsyncDepth = 1;
  param.IOPattern = MFX_IOPATTERN_OUT_SYSTEM_MEMORY;

  //qmfxExtCodingOption ext_coding_option;
  //qmemset(&ext_coding_option, 0, sizeof(ext_coding_option));
  //qext_coding_option.Header.BufferId = MFX_EXTBUFF_CODING_OPTION;
  //qext_coding_option.Header.BufferSz = sizeof(ext_coding_option);
  //qext_coding_option.MaxDecFrameBuffering = 1;

  //qmfxExtBuffer* ext_buffers[1];
  //qext_buffers[0] = (mfxExtBuffer*)&ext_coding_option;
  //qparam.ExtParam = ext_buffers;
  //qparam.NumExtParam = sizeof(ext_buffers) / sizeof(ext_buffers[0]);

  sts = decoder->Query(&param, &param);
  if (sts == MFX_ERR_NONE || sts == MFX_ERR_UNSUPPORTED) {
    return false;
  }

  return true;
}

bool MsdkVideoDecoder::Configure(
    const webrtc::VideoDecoder::Settings& settings) {
  width_ = settings.max_render_resolution().Width();
  height_ = settings.max_render_resolution().Height();

  return InitMediaSDK();
}

int32_t MsdkVideoDecoder::Decode(const webrtc::EncodedImage& input_image,
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

  if (bitstream_.MaxLength < bitstream_.DataLength + input_image.size()) {
    bitstream_buffer_.resize(bitstream_.DataLength + input_image.size());
    bitstream_.MaxLength = bitstream_.DataLength + bitstream_buffer_.size();
    bitstream_.Data = bitstream_buffer_.data();
  }
  //printf("size=%zu\n", input_image.size());
  //for (size_t i = 0; i < input_image.size(); i++) {
  //  const uint8_t* p = input_image.data();
  //  if (i < 100) {
  //    printf(" %02x", p[i]);
  //  } else {
  //    printf("\n");
  //    break;
  //  }
  //}

  memmove(bitstream_.Data, bitstream_.Data + bitstream_.DataOffset,
          bitstream_.DataLength);
  bitstream_.DataOffset = 0;
  memcpy(bitstream_.Data + bitstream_.DataLength, input_image.data(),
         input_image.size());
  bitstream_.DataLength += input_image.size();

  // 使ってない入力サーフェスを取り出す
  auto surface =
      std::find_if(surfaces_.begin(), surfaces_.end(),
                   [](const mfxFrameSurface1& s) { return !s.Data.Locked; });
  if (surface == surfaces_.end()) {
    RTC_LOG(LS_ERROR) << "Surface not found";
    return WEBRTC_VIDEO_CODEC_ERROR;
  }

  // これだとキューイングしたデータとずれるので、本当は surface と一緒に保存して利用するべき
  uint64_t pts = input_image.Timestamp();

  mfxStatus sts;
  mfxSyncPoint syncp;
  mfxFrameSurface1* out_surface = nullptr;
  //RTC_LOG(LS_ERROR) << "before DataOffset=" << bitstream_.DataOffset
  //                  << " DataLength=" << bitstream_.DataLength;
  while (true) {
    sts = decoder_->DecodeFrameAsync(&bitstream_, &*surface, &out_surface,
                                     &syncp);
    if (sts == MFX_WRN_DEVICE_BUSY) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }
    break;
  }
  //RTC_LOG(LS_ERROR) << "after DataOffset=" << bitstream_.DataOffset
  //                  << " DataLength=" << bitstream_.DataLength;
  if (sts == MFX_ERR_MORE_DATA) {
    // もっと入力が必要なので出直す
    return WEBRTC_VIDEO_CODEC_OK;
  }
  if (!syncp) {
    return WEBRTC_VIDEO_CODEC_OK;
  }
  MSDK_CHECK_RESULT(sts, MFX_ERR_NONE, sts);

  sts = session_.SyncOperation(syncp, 600000);
  MSDK_CHECK_RESULT(sts, MFX_ERR_NONE, sts);

  // NV12 から I420 に変換
  rtc::scoped_refptr<webrtc::I420Buffer> i420_buffer =
      buffer_pool_.CreateI420Buffer(width_, height_);
  libyuv::NV12ToI420(out_surface->Data.Y, out_surface->Data.Pitch,
                     out_surface->Data.UV, out_surface->Data.Pitch,
                     i420_buffer->MutableDataY(), i420_buffer->StrideY(),
                     i420_buffer->MutableDataU(), i420_buffer->StrideU(),
                     i420_buffer->MutableDataV(), i420_buffer->StrideV(),
                     width_, height_);

  webrtc::VideoFrame decoded_image = webrtc::VideoFrame::Builder()
                                         .set_video_frame_buffer(i420_buffer)
                                         .set_timestamp_rtp(pts)
                                         .build();
  decode_complete_callback_->Decoded(decoded_image, absl::nullopt,
                                     absl::nullopt);

  return WEBRTC_VIDEO_CODEC_OK;
}

int32_t MsdkVideoDecoder::RegisterDecodeCompleteCallback(
    webrtc::DecodedImageCallback* callback) {
  decode_complete_callback_ = callback;
  return WEBRTC_VIDEO_CODEC_OK;
}

int32_t MsdkVideoDecoder::Release() {
  ReleaseMediaSDK();
  buffer_pool_.Release();
  return WEBRTC_VIDEO_CODEC_OK;
}

const char* MsdkVideoDecoder::ImplementationName() const {
  return "Intel Media SDK";
}

bool MsdkVideoDecoder::InitMediaSDK() {
  decoder_.reset(new MFXVideoDECODE(session_));

  mfxVideoParam param;
  memset(&param, 0, sizeof(param));

  param.mfx.CodecId = codec_id_;
  param.mfx.FrameInfo.FourCC = MFX_FOURCC_NV12;
  param.mfx.FrameInfo.ChromaFormat = MFX_CHROMAFORMAT_YUV420;
  param.mfx.FrameInfo.PicStruct = MFX_PICSTRUCT_PROGRESSIVE;
  param.mfx.FrameInfo.CropX = 0;
  param.mfx.FrameInfo.CropY = 0;
  param.mfx.FrameInfo.CropW = width_;
  param.mfx.FrameInfo.CropH = height_;
  param.mfx.FrameInfo.Width = (width_ + 15) / 16 * 16;
  param.mfx.FrameInfo.Height = (height_ + 15) / 16 * 16;

  param.mfx.GopRefDist = 1;
  param.AsyncDepth = 1;
  param.IOPattern = MFX_IOPATTERN_OUT_SYSTEM_MEMORY;

  //qmfxExtCodingOption ext_coding_option;
  //qmemset(&ext_coding_option, 0, sizeof(ext_coding_option));
  //qext_coding_option.Header.BufferId = MFX_EXTBUFF_CODING_OPTION;
  //qext_coding_option.Header.BufferSz = sizeof(ext_coding_option);
  //qext_coding_option.MaxDecFrameBuffering = 1;

  //qmfxExtBuffer* ext_buffers[1];
  //qext_buffers[0] = (mfxExtBuffer*)&ext_coding_option;
  //qparam.ExtParam = ext_buffers;
  //qparam.NumExtParam = sizeof(ext_buffers) / sizeof(ext_buffers[0]);

  mfxStatus sts = MFX_ERR_NONE;

  sts = decoder_->Query(&param, &param);
  MSDK_CHECK_RESULT(sts, MFX_ERR_NONE, sts);

  // Query number of required surfaces for encoder
  memset(&alloc_request_, 0, sizeof(alloc_request_));
  sts = decoder_->QueryIOSurf(&param, &alloc_request_);
  MSDK_CHECK_RESULT(sts, MFX_ERR_NONE, sts);

  RTC_LOG(LS_INFO) << "Decoder NumFrameSuggested="
                   << alloc_request_.NumFrameSuggested;

  // Initialize the Media SDK encoder
  sts = decoder_->Init(&param);
  //MSDK_IGNORE_MFX_STS(sts, MFX_WRN_PARTIAL_ACCELERATION);
  MSDK_CHECK_RESULT(sts, MFX_ERR_NONE, sts);

  // 入力ビットストリーム
  bitstream_buffer_.resize(1024 * 1024);
  memset(&bitstream_, 0, sizeof(bitstream_));
  bitstream_.MaxLength = bitstream_buffer_.size();
  bitstream_.Data = bitstream_buffer_.data();

  // 必要な枚数分の出力サーフェスを作る
  {
    int width = (alloc_request_.Info.Width + 31) / 32 * 32;
    int height = (alloc_request_.Info.Height + 31) / 32 * 32;
    // 1枚あたりのバイト数
    // NV12 なので 1 ピクセルあたり 12 ビット
    int size = width * height * 12 / 8;
    surface_buffer_.resize(alloc_request_.NumFrameSuggested * size);

    surfaces_.clear();
    surfaces_.reserve(alloc_request_.NumFrameSuggested);
    for (int i = 0; i < alloc_request_.NumFrameSuggested; i++) {
      mfxFrameSurface1 surface;
      memset(&surface, 0, sizeof(surface));
      surface.Info = param.mfx.FrameInfo;
      surface.Data.Y = surface_buffer_.data() + i * size;
      surface.Data.U = surface_buffer_.data() + i * size + width * height;
      surface.Data.V = surface_buffer_.data() + i * size + width * height + 1;
      surface.Data.Pitch = width;
      surfaces_.push_back(surface);
    }
  }

  return true;
}

void MsdkVideoDecoder::ReleaseMediaSDK() {
  decoder_.reset();
}