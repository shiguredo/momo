#ifndef SORA_HWENC_JETSON_JETSON_UTIL_H_
#define SORA_HWENC_JETSON_JETSON_UTIL_H_

namespace sora {

static int VideoCodecToV4L2Format(webrtc::VideoCodecType codec) {
  return codec == webrtc::kVideoCodecVP8    ? V4L2_PIX_FMT_VP8
         : codec == webrtc::kVideoCodecVP9  ? V4L2_PIX_FMT_VP9
         : codec == webrtc::kVideoCodecAV1  ? V4L2_PIX_FMT_AV1
         : codec == webrtc::kVideoCodecH264 ? V4L2_PIX_FMT_H264
         : codec == webrtc::kVideoCodecH265 ? V4L2_PIX_FMT_H265
                                            : 0;
}

}  // namespace sora
#endif
