#ifndef RTC_H264_FORMAT_H_
#define RTC_H264_FORMAT_H_

#include <string>

#include "media/base/codec.h"
#include "media/base/h264_profile_level_id.h"

webrtc::SdpVideoFormat CreateH264Format(webrtc::H264::Profile profile,
                                        webrtc::H264::Level level,
                                        const std::string& packetization_mode);

#endif  // RTC_H264_FORMAT_H_
