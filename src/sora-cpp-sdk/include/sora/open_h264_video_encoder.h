#ifndef SORA_OPEN_H264_VIDEO_ENCODER_H_
#define SORA_OPEN_H264_VIDEO_ENCODER_H_

#include <memory>
#include <string>

// WebRTC
#include <api/video_codecs/video_encoder.h>
#include <media/base/codec.h>

namespace sora {

std::unique_ptr<webrtc::VideoEncoder> CreateOpenH264VideoEncoder(
    const webrtc::SdpVideoFormat& format,
    std::string openh264);

}

#endif
