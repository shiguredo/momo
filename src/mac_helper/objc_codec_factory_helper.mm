/*
 *  Copyright 2017 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "objc_codec_factory_helper.h"

// WebRTC
#include "sdk/objc/native/api/video_decoder_factory.h"
#include "sdk/objc/native/api/video_encoder_factory.h"

// WebRTC
#import "sdk/objc/components/video_codec/RTCDefaultVideoDecoderFactory.h"
#import "sdk/objc/components/video_codec/RTCDefaultVideoEncoderFactory.h"

std::unique_ptr<webrtc::VideoEncoderFactory> CreateObjCEncoderFactory() {
  return webrtc::ObjCToNativeVideoEncoderFactory(
      [[RTCDefaultVideoEncoderFactory alloc] init]);
}

std::unique_ptr<webrtc::VideoDecoderFactory> CreateObjCDecoderFactory() {
  return webrtc::ObjCToNativeVideoDecoderFactory(
      [[RTCDefaultVideoDecoderFactory alloc] init]);
}
