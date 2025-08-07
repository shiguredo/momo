#include "amf_video_encoder.h"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

// WebRTC
#include <api/scoped_refptr.h>
#include <api/video/encoded_image.h>
#include <api/video/render_resolution.h>
#include <api/video/video_codec_type.h>
#include <api/video/video_content_type.h>
#include <api/video/video_frame.h>
#include <api/video/video_frame_buffer.h>
#include <api/video/video_frame_type.h>
#include <api/video/video_rotation.h>
#include <api/video/video_timing.h>
#include <api/video_codecs/scalability_mode.h>
#include <api/video_codecs/video_codec.h>
#include <api/video_codecs/video_encoder.h>
#include <common_video/h264/h264_bitstream_parser.h>
#include <common_video/h265/h265_bitstream_parser.h>
#include <common_video/include/bitrate_adjuster.h>
#include <modules/video_coding/codecs/h264/include/h264_globals.h>
#include <modules/video_coding/include/video_codec_interface.h>
#include <modules/video_coding/include/video_error_codes.h>
#include <modules/video_coding/svc/create_scalability_structure.h>
#include <modules/video_coding/svc/scalable_video_controller.h>
#include <rtc_base/checks.h>
#include <rtc_base/logging.h>

// libyuv
#include <libyuv/convert.h>

// AMF
#include <public/common/AMFFactory.h>
#include <public/common/AMFSTL.h>
#include <public/common/Thread.h>
#include <public/common/TraceAdapter.h>
#include <public/include/components/Component.h>
#include <public/include/components/VideoEncoderAV1.h>
#include <public/include/components/VideoEncoderHEVC.h>
#include <public/include/components/VideoEncoderVCE.h>
#include <public/include/core/Buffer.h>
#include <public/include/core/Context.h>
#include <public/include/core/Data.h>
#include <public/include/core/Plane.h>
#include <public/include/core/Platform.h>
#include <public/include/core/Result.h>
#include <public/include/core/Surface.h>

#include "amf_context_impl.h"
#include "amf_context.h"

#define RETURN_IF_FAILED(res, message)                  \
  if (res != AMF_OK) {                                  \
    RTC_LOG(LS_ERROR) << amf::amf_from_unicode_to_utf8( \
                             amf::AMFFormatResult(res)) \
                      << message;                       \
    return res;                                         \
  }
#define TRACE() RTC_LOG(LS_ERROR) << "TRACE: " << __LINE__

#define FRAME_RTP_TIMESTAMP_PROPERTY L"FRAME_RTP_TIMESTAMP_PROPERTY"
#define FRAME_NTP_TIME_MS_PROPERTY L"FRAME_NTP_TIME_MS_PROPERTY"
#define FRAME_RENDER_TIME_MS_PROPERTY L"FRAME_RENDER_TIME_MS_PROPERTY"
#define FRAME_ROTATION_PROPERTY L"FRAME_ROTATION_PROPERTY"
#define FRAME_COLOR_SPACE_PROPERTY L"FRAME_COLOR_SPACE_PROPERTY"

namespace momo {

const int kLowH264QpThreshold = 34;
const int kHighH264QpThreshold = 40;
#if defined(_WIN32)
const amf::AMF_MEMORY_TYPE kPlatformMemoryType = amf::AMF_MEMORY_DX11;
#else
const amf::AMF_MEMORY_TYPE kPlatformMemoryType = amf::AMF_MEMORY_VULKAN;
#endif

class AMFVideoEncoderImpl : public AMFVideoEncoder {
 public:
  AMFVideoEncoderImpl(std::shared_ptr<AMFContext> amf_context,
                      webrtc::VideoCodecType codec);
  ~AMFVideoEncoderImpl() override;

  static bool IsSupported(std::shared_ptr<AMFContext> amf_context,
                          webrtc::VideoCodecType codec);

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

  static AMF_RESULT CreateEncoder(std::shared_ptr<AMFContext> amf_context,
                                  webrtc::VideoCodecType codec,
                                  int width,
                                  int height,
                                  int framerate,
                                  int target_bitrate_bps,
                                  int max_bitrate_bps,
                                  amf::AMF_MEMORY_TYPE memory_type,
                                  amf::AMFContext** out_context,
                                  amf::AMFComponent** out_encoder);

 private:
  AMF_RESULT InitAMF();
  AMF_RESULT ReleaseAMF();
  AMF_RESULT ProcessBuffer(amf::AMFBufferPtr buffer,
                           webrtc::VideoCodecType codec);

 private:
  std::mutex mutex_;
  webrtc::EncodedImageCallback* callback_ = nullptr;
  webrtc::BitrateAdjuster bitrate_adjuster_;
  uint32_t target_bitrate_bps_ = 0;
  uint32_t max_bitrate_bps_ = 0;

  webrtc::H264BitstreamParser h264_bitstream_parser_;
  webrtc::H265BitstreamParser h265_bitstream_parser_;

  std::shared_ptr<AMFContext> amf_context_;
  webrtc::VideoCodecType codec_;

  bool reconfigure_needed_ = false;
  uint32_t width_ = 0;
  uint32_t height_ = 0;
  uint32_t framerate_ = 0;
  webrtc::VideoCodecMode mode_ = webrtc::VideoCodecMode::kRealtimeVideo;
  std::vector<std::vector<uint8_t>> v_packet_;
  webrtc::EncodedImage encoded_image_;

  // AV1 用
  std::unique_ptr<webrtc::ScalableVideoController> svc_controller_;
  webrtc::ScalabilityMode scalability_mode_;

  amf::AMFContextPtr context_;
  amf::AMFComponentPtr encoder_;
  amf::AMFSurfacePtr surface_;
  amf::AMF_MEMORY_TYPE memory_type_;
  std::unique_ptr<std::thread> polling_thread_;
};

AMFVideoEncoderImpl::AMFVideoEncoderImpl(
    std::shared_ptr<AMFContext> amf_context,
    webrtc::VideoCodecType codec)
    : amf_context_(amf_context), codec_(codec), bitrate_adjuster_(0.5, 0.95) {}

AMFVideoEncoderImpl::~AMFVideoEncoderImpl() {
  Release();
}

int32_t AMFVideoEncoderImpl::InitEncode(
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

  if (codec_settings->codecType == webrtc::kVideoCodecAV1) {
    auto scalability_mode = codec_settings->GetScalabilityMode();
    if (!scalability_mode) {
      RTC_LOG(LS_WARNING) << "Scalability mode is not set, using 'L1T1'.";
      scalability_mode = webrtc::ScalabilityMode::kL1T1;
    }
    RTC_LOG(LS_INFO) << "InitEncode scalability_mode:"
                     << (int)*scalability_mode;
    svc_controller_ = webrtc::CreateScalabilityStructure(*scalability_mode);
    scalability_mode_ = *scalability_mode;
  }

  if (InitAMF() != AMF_OK) {
    return WEBRTC_VIDEO_CODEC_ERROR;
  }
  return WEBRTC_VIDEO_CODEC_OK;
}

int32_t AMFVideoEncoderImpl::RegisterEncodeCompleteCallback(
    webrtc::EncodedImageCallback* callback) {
  std::lock_guard<std::mutex> lock(mutex_);
  callback_ = callback;
  return WEBRTC_VIDEO_CODEC_OK;
}

int32_t AMFVideoEncoderImpl::Release() {
  if (ReleaseAMF() != AMF_OK) {
    return WEBRTC_VIDEO_CODEC_ERROR;
  }
  return WEBRTC_VIDEO_CODEC_OK;
}

int32_t AMFVideoEncoderImpl::Encode(
    const webrtc::VideoFrame& frame,
    const std::vector<webrtc::VideoFrameType>* frame_types) {
  if (encoder_ == nullptr) {
    return WEBRTC_VIDEO_CODEC_UNINITIALIZED;
  }
  if (!callback_) {
    RTC_LOG(LS_WARNING)
        << "InitEncode() has been called, but a callback function "
        << "has not been set with RegisterEncodeCompleteCallback()";
    return WEBRTC_VIDEO_CODEC_UNINITIALIZED;
  }

  AMF_RESULT res;
  if (reconfigure_needed_) {
    if (codec_ == webrtc::kVideoCodecAV1) {
      res = encoder_->SetProperty(AMF_VIDEO_ENCODER_AV1_TARGET_BITRATE,
                                  target_bitrate_bps_);
      RETURN_IF_FAILED(
          res, "Failed to SetProperty(AMF_VIDEO_ENCODER_AV1_TARGET_BITRATE)");
    } else if (codec_ == webrtc::kVideoCodecH265) {
      res = encoder_->SetProperty(AMF_VIDEO_ENCODER_HEVC_TARGET_BITRATE,
                                  target_bitrate_bps_);
      RETURN_IF_FAILED(
          res, "Failed to SetProperty(AMF_VIDEO_ENCODER_HEVC_TARGET_BITRATE)");
    } else if (codec_ == webrtc::kVideoCodecH264) {
      res = encoder_->SetProperty(AMF_VIDEO_ENCODER_TARGET_BITRATE,
                                  target_bitrate_bps_);
      RETURN_IF_FAILED(
          res, "Failed to SetProperty(AMF_VIDEO_ENCODER_TARGET_BITRATE)");
    }
    reconfigure_needed_ = false;
  }

  if (surface_ == nullptr) {
    res = context_->AllocSurface(amf::AMF_MEMORY_HOST, amf::AMF_SURFACE_YUV420P,
                                 width_, height_, &surface_);
    RETURN_IF_FAILED(res, "Failed to AllocSurface");
  }
  {
    amf::AMFPlane* py = surface_->GetPlane(amf::AMF_PLANE_Y);
    amf::AMFPlane* pu = surface_->GetPlane(amf::AMF_PLANE_U);
    amf::AMFPlane* pv = surface_->GetPlane(amf::AMF_PLANE_V);
    webrtc::scoped_refptr<const webrtc::I420BufferInterface> src =
        frame.video_frame_buffer()->ToI420();
    libyuv::I420Copy(src->DataY(), src->StrideY(), src->DataU(), src->StrideU(),
                     src->DataV(), src->StrideV(), (uint8_t*)py->GetNative(),
                     py->GetHPitch(), (uint8_t*)pu->GetNative(),
                     pu->GetHPitch(), (uint8_t*)pv->GetNative(),
                     pv->GetHPitch(), src->width(), src->height());

    res = surface_->SetProperty(FRAME_RTP_TIMESTAMP_PROPERTY,
                                frame.rtp_timestamp());
    RETURN_IF_FAILED(res,
                     "Failed to SetProperty(FRAME_RTP_TIMESTAMP_PROPERTY)");

    res =
        surface_->SetProperty(FRAME_NTP_TIME_MS_PROPERTY, frame.ntp_time_ms());
    RETURN_IF_FAILED(res, "Failed to SetProperty(FRAME_NTP_TIME_MS_PROPERTY)");

    res = surface_->SetProperty(FRAME_RENDER_TIME_MS_PROPERTY,
                                frame.render_time_ms());
    RETURN_IF_FAILED(res,
                     "Failed to SetProperty(FRAME_RENDER_TIME_MS_PROPERTY)");

    res = surface_->SetProperty(FRAME_ROTATION_PROPERTY,
                                (int64_t)frame.rotation());
    RETURN_IF_FAILED(res, "Failed to SetProperty(FRAME_ROTATION_PROPERTY)");
  }

  bool send_key_frame = false;
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

  if (send_key_frame) {
    if (codec_ == webrtc::VideoCodecType::kVideoCodecH264) {
      res = surface_->SetProperty(AMF_VIDEO_ENCODER_FORCE_PICTURE_TYPE,
                                  AMF_VIDEO_ENCODER_PICTURE_TYPE_IDR);
      RETURN_IF_FAILED(
          res, "Failed to SetProperty(AMF_VIDEO_ENCODER_FORCE_PICTURE_TYPE)");
      res = surface_->SetProperty(AMF_VIDEO_ENCODER_INSERT_SPS, true);
      RETURN_IF_FAILED(res,
                       "Failed to SetProperty(AMF_VIDEO_ENCODER_INSERT_SPS)");
      res = surface_->SetProperty(AMF_VIDEO_ENCODER_INSERT_PPS, true);
      RETURN_IF_FAILED(res,
                       "Failed to SetProperty(AMF_VIDEO_ENCODER_INSERT_PPS)");
    } else if (codec_ == webrtc::VideoCodecType::kVideoCodecH265) {
      res = surface_->SetProperty(AMF_VIDEO_ENCODER_HEVC_FORCE_PICTURE_TYPE,
                                  AMF_VIDEO_ENCODER_HEVC_PICTURE_TYPE_IDR);
      RETURN_IF_FAILED(
          res,
          "Failed to SetProperty(AMF_VIDEO_ENCODER_HEVC_FORCE_PICTURE_TYPE)");
      res = surface_->SetProperty(AMF_VIDEO_ENCODER_HEVC_INSERT_HEADER, true);
      RETURN_IF_FAILED(
          res, "Failed to SetProperty(AMF_VIDEO_ENCODER_HEVC_INSERT_HEADER)");
    } else if (codec_ == webrtc::VideoCodecType::kVideoCodecAV1) {
      res = surface_->SetProperty(AMF_VIDEO_ENCODER_AV1_FORCE_FRAME_TYPE,
                                  AMF_VIDEO_ENCODER_AV1_FORCE_FRAME_TYPE_KEY);
      RETURN_IF_FAILED(
          res, "Failed to SetProperty(AMF_VIDEO_ENCODER_AV1_FORCE_FRAME_TYPE)");
      res = surface_->SetProperty(
          AMF_VIDEO_ENCODER_AV1_FORCE_INSERT_SEQUENCE_HEADER, true);
      RETURN_IF_FAILED(
          res,
          "Failed to "
          "SetProperty(AMF_VIDEO_ENCODER_AV1_FORCE_INSERT_SEQUENCE_HEADER)");
    }
  }

  res = encoder_->SubmitInput(surface_);
  if (res == AMF_NEED_MORE_INPUT) {
    // do nothing
  } else if (res == AMF_INPUT_FULL || res == AMF_DECODER_NO_FREE_SURFACES) {
    amf_sleep(1);  // input queue is full: wait, poll and submit again
  } else {
    RETURN_IF_FAILED(res, L"Failed to SubmitInput()");
    surface_ = nullptr;
  }

  return WEBRTC_VIDEO_CODEC_OK;
}

void AMFVideoEncoderImpl::SetRates(
    const webrtc::VideoEncoder::RateControlParameters& parameters) {
  if (encoder_ == nullptr) {
    RTC_LOG(LS_WARNING) << "SetRates() while uninitialized.";
    return;
  }

  if (parameters.framerate_fps < 1.0) {
    RTC_LOG(LS_WARNING) << "Invalid frame rate: " << parameters.framerate_fps;
    return;
  }

  if (svc_controller_) {
    svc_controller_->OnRatesUpdated(parameters.bitrate);
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

webrtc::VideoEncoder::EncoderInfo AMFVideoEncoderImpl::GetEncoderInfo() const {
  webrtc::VideoEncoder::EncoderInfo info;
  info.supports_native_handle = true;
  info.implementation_name = "AMF";
  info.scaling_settings = webrtc::VideoEncoder::ScalingSettings(
      kLowH264QpThreshold, kHighH264QpThreshold);
  return info;
}

AMF_RESULT AMFVideoEncoderImpl::InitAMF() {
  reconfigure_needed_ = false;

  memory_type_ = kPlatformMemoryType;

  auto res = AMFVideoEncoderImpl::CreateEncoder(
      amf_context_, codec_, width_, height_, framerate_, target_bitrate_bps_,
      max_bitrate_bps_, memory_type_, &context_, &encoder_);
  RETURN_IF_FAILED(res, "Failed to CreateEncoder()");

  polling_thread_.reset(new std::thread([this, codec = codec_]() {
    AMF_RESULT res = AMF_OK;
    while (true) {
      amf::AMFDataPtr data;
      res = encoder_->QueryOutput(&data);
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
      amf::AMFBufferPtr buffer(data);
      ProcessBuffer(buffer, codec);
    }
  }));
  return AMF_OK;
}

AMF_RESULT AMFVideoEncoderImpl::ReleaseAMF() {
  if (encoder_ != nullptr) {
    encoder_->Drain();
  }
  if (polling_thread_ != nullptr) {
    polling_thread_->join();
  }

  encoder_ = nullptr;
  polling_thread_.reset();

  return AMF_OK;
}

AMF_RESULT AMFVideoEncoderImpl::ProcessBuffer(amf::AMFBufferPtr buffer,
                                              webrtc::VideoCodecType codec) {
  AMF_RESULT res = AMF_OK;

  uint32_t rtp_timestamp;
  int64_t ntp_time_ms;
  int64_t render_time_ms;
  int64_t rotation;

  res = buffer->GetProperty(FRAME_RTP_TIMESTAMP_PROPERTY, &rtp_timestamp);
  RETURN_IF_FAILED(res, "Failed to GetProperty(FRAME_RTP_TIMESTAMP_PROPERTY)");

  res = buffer->GetProperty(FRAME_NTP_TIME_MS_PROPERTY, &ntp_time_ms);
  RETURN_IF_FAILED(res, "Failed to GetProperty(FRAME_NTP_TIME_MS_PROPERTY)");

  res = buffer->GetProperty(FRAME_RENDER_TIME_MS_PROPERTY, &render_time_ms);
  RETURN_IF_FAILED(res, "Failed to GetProperty(FRAME_RENDER_TIME_MS_PROPERTY)");

  res = buffer->GetProperty(FRAME_ROTATION_PROPERTY, &rotation);
  RETURN_IF_FAILED(res, "Failed to GetProperty(FRAME_ROTATION_PROPERTY)");

  uint8_t* p = (uint8_t*)buffer->GetNative();
  size_t size = buffer->GetSize();
  auto encoded_image_buffer = webrtc::EncodedImageBuffer::Create(p, size);

  encoded_image_.SetEncodedData(encoded_image_buffer);
  encoded_image_._encodedWidth = width_;
  encoded_image_._encodedHeight = height_;
  encoded_image_.content_type_ =
      (mode_ == webrtc::VideoCodecMode::kScreensharing)
          ? webrtc::VideoContentType::SCREENSHARE
          : webrtc::VideoContentType::UNSPECIFIED;
  encoded_image_.timing_.flags = webrtc::VideoSendTiming::kInvalid;
  encoded_image_.SetRtpTimestamp(rtp_timestamp);
  encoded_image_.ntp_time_ms_ = ntp_time_ms;
  encoded_image_.capture_time_ms_ = render_time_ms;
  encoded_image_.rotation_ = (webrtc::VideoRotation)rotation;
  encoded_image_._frameType = webrtc::VideoFrameType::kVideoFrameDelta;

  // キーフレーム判定
  // コーデック毎に使うプロパティが違う
  if (codec == webrtc::kVideoCodecH264) {
    int64_t data_type;
    res = buffer->GetProperty(AMF_VIDEO_ENCODER_OUTPUT_DATA_TYPE, &data_type);
    RETURN_IF_FAILED(
        res, "Failed to GetProperty(AMF_VIDEO_ENCODER_OUTPUT_DATA_TYPE)");
    if (data_type == AMF_VIDEO_ENCODER_OUTPUT_DATA_TYPE_IDR ||
        data_type == AMF_VIDEO_ENCODER_OUTPUT_DATA_TYPE_I) {
      encoded_image_._frameType = webrtc::VideoFrameType::kVideoFrameKey;
    }
  } else if (codec == webrtc::kVideoCodecH265) {
    int64_t data_type;
    res = buffer->GetProperty(AMF_VIDEO_ENCODER_HEVC_OUTPUT_DATA_TYPE,
                              &data_type);
    RETURN_IF_FAILED(
        res, "Failed to GetProperty(AMF_VIDEO_ENCODER_HEVC_OUTPUT_DATA_TYPE)");
    if (data_type == AMF_VIDEO_ENCODER_HEVC_OUTPUT_DATA_TYPE_IDR ||
        data_type == AMF_VIDEO_ENCODER_HEVC_OUTPUT_DATA_TYPE_I) {
      encoded_image_._frameType = webrtc::VideoFrameType::kVideoFrameKey;
    }
  } else if (codec == webrtc::kVideoCodecAV1) {
    int64_t data_type;
    res = buffer->GetProperty(AMF_VIDEO_ENCODER_AV1_OUTPUT_FRAME_TYPE,
                              &data_type);
    RETURN_IF_FAILED(
        res, "Failed to GetProperty(AMF_VIDEO_ENCODER_AV1_OUTPUT_FRAME_TYPE)");
    if (data_type == AMF_VIDEO_ENCODER_AV1_OUTPUT_FRAME_TYPE_KEY) {
      encoded_image_._frameType = webrtc::VideoFrameType::kVideoFrameKey;
    }
  }

  webrtc::CodecSpecificInfo codec_specific;
  if (codec_ == webrtc::kVideoCodecH264) {
    codec_specific.codecType = webrtc::kVideoCodecH264;
    codec_specific.codecSpecific.H264.packetization_mode =
        webrtc::H264PacketizationMode::NonInterleaved;

    h264_bitstream_parser_.ParseBitstream(encoded_image_);
    encoded_image_.qp_ = h264_bitstream_parser_.GetLastSliceQp().value_or(-1);
  } else if (codec_ == webrtc::kVideoCodecH265) {
    codec_specific.codecType = webrtc::kVideoCodecH265;

    h265_bitstream_parser_.ParseBitstream(encoded_image_);
    encoded_image_.qp_ = h265_bitstream_parser_.GetLastSliceQp().value_or(-1);
  } else if (codec_ == webrtc::kVideoCodecAV1) {
    codec_specific.codecType = webrtc::kVideoCodecAV1;

    bool is_key =
        encoded_image_._frameType == webrtc::VideoFrameType::kVideoFrameKey;
    std::vector<webrtc::ScalableVideoController::LayerFrameConfig>
        layer_frames = svc_controller_->NextFrameConfig(is_key);
    // AV1 の SVC では、まれにエンコード対象のレイヤーフレームが存在しない場合がある。
    // 次のフレームを待つことで正常に継続可能なケースであるため、エラーではなく正常終了の AMF_OK を返してスキップする。
    if (layer_frames.empty()) {
      return AMF_OK;
    }
    codec_specific.end_of_picture = true;
    codec_specific.scalability_mode = scalability_mode_;
    codec_specific.generic_frame_info =
        svc_controller_->OnEncodeDone(layer_frames[0]);
    if (is_key && codec_specific.generic_frame_info) {
      codec_specific.template_structure =
          svc_controller_->DependencyStructure();
      auto& resolutions = codec_specific.template_structure->resolutions;
      resolutions = {webrtc::RenderResolution(encoded_image_._encodedWidth,
                                              encoded_image_._encodedHeight)};
    }
  }

  webrtc::EncodedImageCallback::Result result =
      callback_->OnEncodedImage(encoded_image_, &codec_specific);
  if (result.error != webrtc::EncodedImageCallback::Result::OK) {
    RTC_LOG(LS_ERROR) << __FUNCTION__
                      << " OnEncodedImage failed error:" << result.error;
    return AMF_FAIL;
  }
  bitrate_adjuster_.Update(size);

  return AMF_OK;
}

AMF_RESULT AMFVideoEncoderImpl::CreateEncoder(
    std::shared_ptr<AMFContext> amf_context,
    webrtc::VideoCodecType codec,
    int width,
    int height,
    int framerate,
    int target_bitrate_bps,
    int max_bitrate_bps,
    amf::AMF_MEMORY_TYPE memory_type,
    amf::AMFContext** out_context,
    amf::AMFComponent** out_encoder) {
  if (!(codec == webrtc::kVideoCodecAV1 || codec == webrtc::kVideoCodecH265 ||
        codec == webrtc::kVideoCodecH264)) {
    return AMF_NOT_SUPPORTED;
  }

  AMF_RESULT res = AMF_OK;

  amf::AMFContextPtr context;
  amf::AMFComponentPtr encoder;
  amf::AMFSurfacePtr surface;
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

  auto amf_codec = codec == webrtc::kVideoCodecH264   ? AMFVideoEncoderVCE_AVC
                   : codec == webrtc::kVideoCodecH265 ? AMFVideoEncoder_HEVC
                   : codec == webrtc::kVideoCodecAV1  ? AMFVideoEncoder_AV1
                                                      : L"";
  res = GetAMFFactoryHelper(amf_context)
            ->GetFactory()
            ->CreateComponent(context, amf_codec, &encoder);
  RETURN_IF_FAILED(res, L"CreateComponent() failed");

  if (codec == webrtc::kVideoCodecAV1) {
    res = encoder->SetProperty(AMF_VIDEO_ENCODER_AV1_USAGE,
                               AMF_VIDEO_ENCODER_AV1_USAGE_TRANSCODING);
    RETURN_IF_FAILED(res,
                     "Failed to SetProperty(AMF_VIDEO_ENCODER_AV1_USAGE, "
                     "AMF_VIDEO_ENCODER_AV1_USAGE_TRANSCODING)");

    res = encoder->SetProperty(
        AMF_VIDEO_ENCODER_AV1_ENCODING_LATENCY_MODE,
        AMF_VIDEO_ENCODER_AV1_ENCODING_LATENCY_MODE_LOWEST_LATENCY);
    RETURN_IF_FAILED(
        res,
        "SetProperty(AMF_VIDEO_ENCODER_AV1_ENCODING_LATENCY_MODE, "
        "AMF_VIDEO_ENCODER_AV1_ENCODING_LATENCY_MODE_LOWEST_LATENCY) failed");

    res = encoder->SetProperty(
        AMF_VIDEO_ENCODER_AV1_ALIGNMENT_MODE,
        AMF_VIDEO_ENCODER_AV1_ALIGNMENT_MODE_NO_RESTRICTIONS);
    RETURN_IF_FAILED(
        res,
        "Failed to SetProperty(AMF_VIDEO_ENCODER_AV1_ALIGNMENT_MODE, "
        "AMF_VIDEO_ENCODER_AV1_ALIGNMENT_MODE_NO_RESTRICTIONS)");

    res = encoder->SetProperty(AMF_VIDEO_ENCODER_AV1_TARGET_BITRATE,
                               target_bitrate_bps);
    RETURN_IF_FAILED(
        res, "Failed to SetProperty(AMF_VIDEO_ENCODER_AV1_TARGET_BITRATE)");

    res = encoder->SetProperty(AMF_VIDEO_ENCODER_AV1_FRAMERATE,
                               ::AMFConstructRate(framerate, 1));
    RETURN_IF_FAILED(res,
                     "Failed to SetProperty(AMF_VIDEO_ENCODER_AV1_FRAMERATE)");

    res = encoder->SetProperty(AMF_VIDEO_ENCODER_AV1_FRAMESIZE,
                               ::AMFConstructSize(width, height));
    RETURN_IF_FAILED(res,
                     "Failed to SetProperty(AMF_VIDEO_ENCODER_AV1_FRAMESIZE");
  } else if (codec == webrtc::kVideoCodecH265) {
    res = encoder->SetProperty(AMF_VIDEO_ENCODER_HEVC_USAGE,
                               AMF_VIDEO_ENCODER_HEVC_USAGE_TRANSCODING);
    RETURN_IF_FAILED(res,
                     "Failed to SetProperty(AMF_VIDEO_ENCODER_HEVC_USAGE, "
                     "AMF_VIDEO_ENCODER_HEVC_USAGE_TRANSCODING)");

    res = encoder->SetProperty(AMF_VIDEO_ENCODER_HEVC_TARGET_BITRATE,
                               target_bitrate_bps);
    RETURN_IF_FAILED(
        res, "Failed to SetProperty(AMF_VIDEO_ENCODER_HEVC_TARGET_BITRATE)");

    res = encoder->SetProperty(AMF_VIDEO_ENCODER_HEVC_FRAMERATE,
                               ::AMFConstructRate(framerate, 1));
    RETURN_IF_FAILED(res,
                     "Failed to SetProperty(AMF_VIDEO_ENCODER_HEVC_FRAMERATE)");

    res = encoder->SetProperty(AMF_VIDEO_ENCODER_HEVC_LOWLATENCY_MODE, true);
    RETURN_IF_FAILED(
        res,
        "Failed to SetProperty(AMF_VIDEO_ENCODER_HEVC_LOWLATENCY_MODE, true)");
  } else if (codec == webrtc::kVideoCodecH264) {
    res = encoder->SetProperty(AMF_VIDEO_ENCODER_USAGE,
                               AMF_VIDEO_ENCODER_USAGE_TRANSCODING);
    RETURN_IF_FAILED(res,
                     "Failed to SetProperty(AMF_VIDEO_ENCODER_USAGE, "
                     "AMF_VIDEO_ENCODER_USAGE_TRANSCODING)");

    res = encoder->SetProperty(AMF_VIDEO_ENCODER_TARGET_BITRATE,
                               target_bitrate_bps);
    RETURN_IF_FAILED(res,
                     "Failed to SetProperty(AMF_VIDEO_ENCODER_TARGET_BITRATE)");
    res = encoder->SetProperty(AMF_VIDEO_ENCODER_FRAMERATE,
                               ::AMFConstructRate(framerate, 1));
    RETURN_IF_FAILED(res, "Failed to SetProperty(AMF_VIDEO_ENCODER_FRAMERATE");

    res = encoder->SetProperty(AMF_VIDEO_ENCODER_LOWLATENCY_MODE, true);
    RETURN_IF_FAILED(
        res, "Failed to SetProperty(AMF_VIDEO_ENCODER_LOWLATENCY_MODE, true)");
  }

  res = encoder->Init(amf::AMF_SURFACE_YUV420P, width, height);
  RETURN_IF_FAILED(res, "Failed to encoder->Init()");

  *out_context = context.Detach();
  *out_encoder = encoder.Detach();

  return res;
}

bool AMFVideoEncoder::IsSupported(std::shared_ptr<AMFContext> amf_context,
                                  webrtc::VideoCodecType codec) {
  if (amf_context == nullptr) {
    return false;
  }

  amf::AMFContextPtr context;
  amf::AMFComponentPtr encoder;
  auto res = AMFVideoEncoderImpl::CreateEncoder(
      amf_context, codec, 640, 480, 30, 100 * 1000, 500 * 1000,
      kPlatformMemoryType, &context, &encoder);
  if (res != AMF_OK || context == nullptr || encoder == nullptr) {
    return false;
  }
  return true;
}

std::unique_ptr<AMFVideoEncoder> AMFVideoEncoder::Create(
    std::shared_ptr<AMFContext> amf_context,
    webrtc::VideoCodecType codec) {
  return std::unique_ptr<AMFVideoEncoder>(
      new AMFVideoEncoderImpl(amf_context, codec));
}

}  // namespace momo