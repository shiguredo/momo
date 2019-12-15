#include "h264_format.h"

// webrtc
#include "absl/types/optional.h"
#include "api/video_codecs/sdp_video_format.h"

// modules/video_coding/codecs/h264/h264.cc より
webrtc::SdpVideoFormat CreateH264Format(webrtc::H264::Profile profile,
                                        webrtc::H264::Level level,
                                        const std::string& packetization_mode) {
  const absl::optional<std::string> profile_string =
      webrtc::H264::ProfileLevelIdToString(
          webrtc::H264::ProfileLevelId(profile, level));
  return webrtc::SdpVideoFormat(
      cricket::kH264CodecName,
      {{cricket::kH264FmtpProfileLevelId, *profile_string},
       {cricket::kH264FmtpLevelAsymmetryAllowed, "1"},
       {cricket::kH264FmtpPacketizationMode, packetization_mode}});
}
