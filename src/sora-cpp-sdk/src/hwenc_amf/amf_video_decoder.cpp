#include "sora/hwenc_amf/amf_video_decoder.h"

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <optional>
#include <thread>

// WebRTC
#include <api/scoped_refptr.h>
#include <api/video/encoded_image.h>
#include <api/video/i420_buffer.h>
#include <api/video/video_codec_type.h>
#include <api/video/video_frame.h>
#include <api/video_codecs/video_codec.h>
#include <api/video_codecs/video_decoder.h>
#include <common_video/include/video_frame_buffer_pool.h>
#include <modules/video_coding/include/video_error_codes.h>
#include <rtc_base/logging.h>

// libyuv
#include <libyuv/convert.h>

// AMF
#include <public/common/AMFFactory.h>
#include <public/common/AMFSTL.h>
#include <public/common/Thread.h>
#include <public/common/TraceAdapter.h>
#include <public/include/components/Component.h>
#include <public/include/components/VideoDecoderUVD.h>
#include <public/include/core/Buffer.h>
#include <public/include/core/Context.h>
#include <public/include/core/Data.h>
#include <public/include/core/Plane.h>
#include <public/include/core/Result.h>
#include <public/include/core/Surface.h>

#include "../amf_context_impl.h"
#include "sora/amf_context.h"

#define RETURN_IF_FAILED(res, message)                  \
  if (res != AMF_OK) {                                  \
    RTC_LOG(LS_ERROR) << amf::amf_from_unicode_to_utf8( \
                             amf::AMFFormatResult(res)) \
                      << message;                       \
    return res;                                         \
  }
#define TRACE() RTC_LOG(LS_ERROR) << "TRACE: " << __LINE__

namespace sora {

#if defined(_WIN32)
const amf::AMF_MEMORY_TYPE kPlatformMemoryType = amf::AMF_MEMORY_DX11;
#else
const amf::AMF_MEMORY_TYPE kPlatformMemoryType = amf::AMF_MEMORY_VULKAN;
#endif

class AMFVideoDecoderImpl : public AMFVideoDecoder {
 public:
  AMFVideoDecoderImpl(std::shared_ptr<AMFContext> amf_context,
                      webrtc::VideoCodecType codec);
  ~AMFVideoDecoderImpl() override;

  static bool IsSupported(std::shared_ptr<AMFContext> amf_context,
                          webrtc::VideoCodecType codec);

  bool Configure(const Settings& settings) override;
  int32_t Decode(const webrtc::EncodedImage& input_image,
                 bool missing_frames,
                 int64_t render_time_ms) override;
  int32_t RegisterDecodeCompleteCallback(
      webrtc::DecodedImageCallback* callback) override;
  int32_t Release() override;
  const char* ImplementationName() const override;

 private:
  static AMF_RESULT CreateDecoder(std::shared_ptr<AMFContext> amf_context,
                                  webrtc::VideoCodecType codec,
                                  amf::AMF_MEMORY_TYPE memory_type,
                                  amf::AMFContext** out_context,
                                  amf::AMFComponent** out_decoder);
  AMF_RESULT InitAMF();
  void ReleaseAMF();
  AMF_RESULT ProcessSurface(amf::AMFSurfacePtr surface);

  webrtc::DecodedImageCallback* decode_complete_callback_ = nullptr;
  webrtc::VideoFrameBufferPool buffer_pool_;

  std::shared_ptr<AMFContext> amf_context_;
  webrtc::VideoCodecType codec_;

  amf::AMFContextPtr context_;
  amf::AMFComponentPtr decoder_;
  amf::AMF_MEMORY_TYPE memory_type_;
  std::unique_ptr<std::thread> polling_thread_;
};

AMFVideoDecoderImpl::AMFVideoDecoderImpl(
    std::shared_ptr<AMFContext> amf_context,
    webrtc::VideoCodecType codec)
    : amf_context_(amf_context),
      codec_(codec),
      decode_complete_callback_(nullptr),
      buffer_pool_(false, 300 /* max_number_of_buffers*/) {}

AMFVideoDecoderImpl::~AMFVideoDecoderImpl() {
  Release();
}

bool AMFVideoDecoder::IsSupported(std::shared_ptr<AMFContext> amf_context,
                                  webrtc::VideoCodecType codec) {
  RTC_LOG(LS_ERROR) << "AMFVideoDecoder::IsSupported: "
                    << webrtc::CodecTypeToPayloadString(codec);
  return AMFVideoDecoderImpl::IsSupported(amf_context, codec);
}

bool AMFVideoDecoderImpl::IsSupported(std::shared_ptr<AMFContext> amf_context,
                                      webrtc::VideoCodecType codec) {
  if (amf_context == nullptr) {
    return false;
  }

  amf::AMFContextPtr context;
  amf::AMFComponentPtr decoder;
  AMF_RESULT res = CreateDecoder(amf_context, codec, kPlatformMemoryType,
                                 &context, &decoder);
  if (res != AMF_OK) {
    return false;
  }
  return true;
}

bool AMFVideoDecoderImpl::Configure(const Settings& settings) {
  if (InitAMF() != AMF_OK) {
    return false;
  }
  return true;
}

int32_t AMFVideoDecoderImpl::Decode(const webrtc::EncodedImage& input_image,
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

  AMF_RESULT res;
  amf::AMFBufferPtr buffer;
  res =
      context_->AllocBuffer(amf::AMF_MEMORY_HOST, input_image.size(), &buffer);
  RETURN_IF_FAILED(res, "Failed to AllocBuffer()");

  memcpy(buffer->GetNative(), input_image.data(), input_image.size());

  buffer->SetPts(input_image.RtpTimestamp());

  while (true) {
    if (res == AMF_REPEAT) {
      res = decoder_->SubmitInput(nullptr);
    } else {
      res = decoder_->SubmitInput(buffer);
    }
    if (res == AMF_NEED_MORE_INPUT) {
      return WEBRTC_VIDEO_CODEC_OK;
    } else if (res == AMF_INPUT_FULL || res == AMF_DECODER_NO_FREE_SURFACES ||
               res == AMF_REPEAT) {
      // queue is full; sleep, try to get ready surfaces  in polling thread and repeat submission
      amf_sleep(1);
      continue;
    } else if (res == AMF_RESOLUTION_CHANGED || res == AMF_RESOLUTION_UPDATED) {
      // デコードするサイズが変わったらデコーダを作り直す
      ReleaseAMF();
      InitAMF();
      continue;
    } else {
      RETURN_IF_FAILED(res, L"Failed to SubmitInput()");
      break;
    }
  }

  return WEBRTC_VIDEO_CODEC_OK;
}

int32_t AMFVideoDecoderImpl::RegisterDecodeCompleteCallback(
    webrtc::DecodedImageCallback* callback) {
  decode_complete_callback_ = callback;
  return WEBRTC_VIDEO_CODEC_OK;
}

int32_t AMFVideoDecoderImpl::Release() {
  ReleaseAMF();
  buffer_pool_.Release();
  return WEBRTC_VIDEO_CODEC_OK;
}

AMF_RESULT AMFVideoDecoderImpl::ProcessSurface(amf::AMFSurfacePtr surface) {
  AMF_RESULT res = AMF_OK;
  res = surface->Convert(amf::AMF_MEMORY_HOST);
  RETURN_IF_FAILED(res, "Failed to Convert()");

  uint32_t pts = surface->GetPts();

  auto py = surface->GetPlane(amf::AMF_PLANE_Y);
  auto pu = surface->GetPlane(amf::AMF_PLANE_U);
  auto pv = surface->GetPlane(amf::AMF_PLANE_V);

  // RTC_LOG(LS_ERROR) << "AMFVideoDecoderImpl::ProcessSurface: "
  //                   << "pts=" << pts << ", width=" << py->GetWidth()
  //                   << ", height=" << py->GetHeight()
  //                   << ", pitch_y=" << py->GetHPitch()
  //                   << ", pitch_u=" << pu->GetHPitch()
  //                   << ", pitch_v=" << pv->GetHPitch();
  webrtc::scoped_refptr<webrtc::I420Buffer> i420_buffer =
      buffer_pool_.CreateI420Buffer(py->GetWidth(), py->GetHeight());
  libyuv::I420Copy((const uint8_t*)py->GetNative(), py->GetHPitch(),
                   (const uint8_t*)pu->GetNative(), pu->GetHPitch(),
                   (const uint8_t*)pv->GetNative(), pv->GetHPitch(),
                   i420_buffer->MutableDataY(), i420_buffer->StrideY(),
                   i420_buffer->MutableDataU(), i420_buffer->StrideU(),
                   i420_buffer->MutableDataV(), i420_buffer->StrideV(),
                   py->GetWidth(), py->GetHeight());

  webrtc::VideoFrame decoded_image = webrtc::VideoFrame::Builder()
                                         .set_video_frame_buffer(i420_buffer)
                                         .set_timestamp_rtp(pts)
                                         .build();
  decode_complete_callback_->Decoded(decoded_image, std::nullopt, std::nullopt);

  return res;
}

const char* AMFVideoDecoderImpl::ImplementationName() const {
  return "AMF";
}

AMF_RESULT AMFVideoDecoderImpl::CreateDecoder(
    std::shared_ptr<AMFContext> amf_context,
    webrtc::VideoCodecType codec,
    amf::AMF_MEMORY_TYPE memory_type,
    amf::AMFContext** out_context,
    amf::AMFComponent** out_decoder) {
  if (!(codec == webrtc::kVideoCodecVP9 || codec == webrtc::kVideoCodecH264 ||
        codec == webrtc::kVideoCodecH265 || codec == webrtc::kVideoCodecAV1)) {
    return AMF_NOT_SUPPORTED;
  }

  AMF_RESULT res = AMF_OK;

  amf::AMFContextPtr context;
  amf::AMFComponentPtr decoder;
  res = GetAMFFactoryHelper(amf_context)->GetFactory()->CreateContext(&context);
  RETURN_IF_FAILED(res, "Failed to CreateContext()");
  if (memory_type == amf::AMF_MEMORY_OPENGL) {
    res = context->InitOpenGL(NULL, NULL, NULL);
    RETURN_IF_FAILED(res, "Failed to InitOpenGL()");
  } else if (memory_type == amf::AMF_MEMORY_VULKAN) {
    res = amf::AMFContext1Ptr(context)->InitVulkan(NULL);
    RETURN_IF_FAILED(res, "Failed to InitVulkan()");
  }
#if defined(_WIN32)
  if (memory_type == amf::AMF_MEMORY_DX9) {
    res = context->InitDX9(NULL);  // can be DX9 or DX9Ex device
    RETURN_IF_FAILED(res, "Failed to InitDX9(NULL)");
  } else if (memory_type == amf::AMF_MEMORY_DX11) {
    res = context->InitDX11(NULL);  // can be DX11 device
    RETURN_IF_FAILED(res, "Failed to InitDX11(NULL)");
  } else if (memory_type == amf::AMF_MEMORY_DX12) {
    res = amf::AMFContext2Ptr(context)->InitDX12(NULL);  // can be DX12 device
    RETURN_IF_FAILED(res, "Failed to InitDX12(NULL)");
  }
#endif

  auto codec_name =
      codec == webrtc::kVideoCodecVP9    ? AMFVideoDecoderHW_VP9
      : codec == webrtc::kVideoCodecH264 ? AMFVideoDecoderUVD_H264_AVC
      : codec == webrtc::kVideoCodecH265 ? AMFVideoDecoderHW_H265_HEVC
      : codec == webrtc::kVideoCodecAV1  ? AMFVideoDecoderHW_AV1
                                         : L"";
  res = GetAMFFactoryHelper(amf_context)
            ->GetFactory()
            ->CreateComponent(context, codec_name, &decoder);
  RETURN_IF_FAILED(res, L"CreateComponent() failed");

  res = decoder->Init(amf::AMF_SURFACE_YUV420P, 4096, 4096);
  RETURN_IF_FAILED(res, L"Init() failed");

  *out_context = context.Detach();
  *out_decoder = decoder.Detach();

  return AMF_OK;
}

AMF_RESULT AMFVideoDecoderImpl::InitAMF() {
  AMF_RESULT res = AMF_OK;

  memory_type_ = kPlatformMemoryType;

  res = CreateDecoder(amf_context_, codec_, memory_type_, &context_, &decoder_);
  RETURN_IF_FAILED(res, "Failed to CreateDecoder()");

  polling_thread_.reset(new std::thread([this]() {
    AMF_RESULT res = AMF_OK;
    while (true) {
      amf::AMFDataPtr data;
      res = decoder_->QueryOutput(&data);
      if (res == AMF_EOF) {
        break;  // Drain complete
      }
      if ((res != AMF_OK) && (res != AMF_REPEAT)) {
        // trace possible error message
        break;  // Drain complete
      }
      if (data == nullptr) {
        amf_sleep(1);
        continue;
      }
      amf::AMFSurfacePtr surface(data);
      ProcessSurface(surface);
    }
  }));
  return res;
}

void AMFVideoDecoderImpl::ReleaseAMF() {
  if (decoder_ != nullptr) {
    decoder_->Drain();
  }
  if (polling_thread_ != nullptr) {
    polling_thread_->join();
  }
  decoder_ = nullptr;
  polling_thread_.reset();
}

std::unique_ptr<AMFVideoDecoder> AMFVideoDecoder::Create(
    std::shared_ptr<AMFContext> amf_context,
    webrtc::VideoCodecType codec) {
  return std::make_unique<AMFVideoDecoderImpl>(amf_context, codec);
}

}  // namespace sora
