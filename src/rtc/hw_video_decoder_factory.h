/*
 *  Copyright (c) 2016 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef HW_VIDEO_DECODER_FACTORY_H_
#define HW_VIDEO_DECODER_FACTORY_H_

#include <memory>
#include <vector>

#include "api/video_codecs/video_decoder_factory.h"

class HWVideoDecoderFactory : public webrtc::VideoDecoderFactory {
 public:
  HWVideoDecoderFactory() {}
  virtual ~HWVideoDecoderFactory() {}

  std::vector<webrtc::SdpVideoFormat> GetSupportedFormats() const override;

  std::unique_ptr<webrtc::VideoDecoder> CreateVideoDecoder(
      const webrtc::SdpVideoFormat& format) override;
};

#endif  // HW_VIDEO_DECODER_FACTORY_H_