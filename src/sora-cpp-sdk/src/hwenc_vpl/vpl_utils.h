#ifndef SORA_HWENC_VPL_VPL_UTILS_H_
#define SORA_HWENC_VPL_VPL_UTILS_H_

// WebRTC
#include <api/video/video_codec_type.h>

// Intel VPL
#include <vpl/mfxdefs.h>

#define VPL_CHECK_RESULT(P, X, ERR)                    \
  {                                                    \
    if ((X) > (P)) {                                   \
      RTC_LOG(LS_ERROR) << "Intel VPL Error: " << ERR; \
      throw ERR;                                       \
    }                                                  \
  }

namespace sora {

static mfxU32 ToMfxCodec(webrtc::VideoCodecType codec) {
  return codec == webrtc::kVideoCodecVP8    ? MFX_CODEC_VP8
         : codec == webrtc::kVideoCodecVP9  ? MFX_CODEC_VP9
         : codec == webrtc::kVideoCodecH264 ? MFX_CODEC_AVC
         : codec == webrtc::kVideoCodecH265 ? MFX_CODEC_HEVC
                                            : MFX_CODEC_AV1;
}

static std::string CodecToString(mfxU32 codec) {
  return codec == MFX_CODEC_VP8    ? "MFX_CODEC_VP8"
         : codec == MFX_CODEC_VP9  ? "MFX_CODEC_VP9"
         : codec == MFX_CODEC_AV1  ? "MFX_CODEC_AV1"
         : codec == MFX_CODEC_AVC  ? "MFX_CODEC_AVC"
         : codec == MFX_CODEC_HEVC ? "MFX_CODEC_HEVC"
                                   : "MFX_CODEC_UNKNOWN";
}

}  // namespace sora
#endif
