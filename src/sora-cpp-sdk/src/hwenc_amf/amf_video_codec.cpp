#include "sora/hwenc_amf/amf_video_codec.h"

#include <memory>
#include <string>

// WebRTC
#include <api/video/video_codec_type.h>

// AMF
#include <public/include/core/Version.h>

#include "../amf_context_impl.h"
#include "sora/amf_context.h"
#include "sora/hwenc_amf/amf_video_decoder.h"
#include "sora/hwenc_amf/amf_video_encoder.h"
#include "sora/sora_video_codec.h"

namespace sora {

VideoCodecCapability::Engine GetAMFVideoCodecCapability(
    std::shared_ptr<AMFContext> context) {
  VideoCodecCapability::Engine engine(VideoCodecImplementation::kAmdAmf);
  if (context == nullptr) {
    return engine;
  }

  auto runtime_version = GetAMFFactoryHelper(context)->AMFQueryVersion();
  engine.parameters.amf_runtime_version =
      std::to_string(AMF_GET_MAJOR_VERSION(runtime_version)) + "." +
      std::to_string(AMF_GET_MINOR_VERSION(runtime_version)) + "." +
      std::to_string(AMF_GET_SUBMINOR_VERSION(runtime_version)) + "." +
      std::to_string(AMF_GET_BUILD_VERSION(runtime_version));
  engine.parameters.amf_embedded_version =
      std::to_string(AMF_VERSION_MAJOR) + "." +
      std::to_string(AMF_VERSION_MINOR) + "." +
      std::to_string(AMF_VERSION_RELEASE) + "." +
      std::to_string(AMF_VERSION_BUILD_NUM);

  auto add = [&engine, &context](webrtc::VideoCodecType type) {
    engine.codecs.emplace_back(type,
                               AMFVideoEncoder::IsSupported(context, type),
                               AMFVideoDecoder::IsSupported(context, type));
  };
  add(webrtc::kVideoCodecVP8);
  add(webrtc::kVideoCodecVP9);
  add(webrtc::kVideoCodecH264);
  add(webrtc::kVideoCodecH265);
  add(webrtc::kVideoCodecAV1);
  return engine;
}

}  // namespace sora
