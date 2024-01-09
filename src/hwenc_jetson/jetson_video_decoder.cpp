/*
 *  Copyright (c) 2015 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 *
 */

#include "jetson_video_decoder.h"

#include <unistd.h>

// WebRTC
#include <modules/video_coding/include/video_error_codes.h>
#include <rtc_base/checks.h>
#include <rtc_base/logging.h>
#include <rtc_base/time_utils.h>
#include <system_wrappers/include/metrics.h>
#include <third_party/libyuv/include/libyuv/convert.h>

// L4T Multimedia API
#include <nvbufsurface.h>
#include <nvbufsurftransform.h>

// Jetson Linux Multimedia API
#include <NvBufSurface.h>
#include <NvVideoDecoder.h>

#define INIT_ERROR(cond, desc)                 \
  if (cond) {                                  \
    RTC_LOG(LS_ERROR) << __FUNCTION__ << desc; \
    Release();                                 \
    return WEBRTC_VIDEO_CODEC_ERROR;           \
  }
#define CHUNK_SIZE 4000000

JetsonVideoDecoder::JetsonVideoDecoder(webrtc::VideoCodecType codec)
    : input_format_(codec == webrtc::kVideoCodecVP8    ? V4L2_PIX_FMT_VP8
                    : codec == webrtc::kVideoCodecVP9  ? V4L2_PIX_FMT_VP9
                    : codec == webrtc::kVideoCodecH264 ? V4L2_PIX_FMT_H264
                    : codec == webrtc::kVideoCodecAV1  ? V4L2_PIX_FMT_AV1
                                                       : 0),
      decoder_(nullptr),
      decode_complete_callback_(nullptr),
      buffer_pool_(false, 300 /* max_number_of_buffers*/),
      eos_(false),
      got_error_(false),
      dst_dma_fd_(-1) {}

JetsonVideoDecoder::~JetsonVideoDecoder() {
  Release();
}

bool JetsonVideoDecoder::IsSupportedVP8() {
  //SuppressErrors sup;

  auto decoder = NvVideoDecoder::createVideoDecoder("dec0");
  auto ret = decoder->setOutputPlaneFormat(V4L2_PIX_FMT_VP8, CHUNK_SIZE);
  delete decoder;
  return ret >= 0;
}

bool JetsonVideoDecoder::IsSupportedAV1() {
  //SuppressErrors sup;

  auto decoder = NvVideoDecoder::createVideoDecoder("dec0");
  auto ret = decoder->setOutputPlaneFormat(V4L2_PIX_FMT_AV1, CHUNK_SIZE);
  delete decoder;
  return ret >= 0;
}

bool JetsonVideoDecoder::Configure(const Settings& settings) {
  if (JetsonConfigure() != WEBRTC_VIDEO_CODEC_OK) {
    RTC_LOG(LS_ERROR) << __FUNCTION__ << "Failed to JetsonConfigure";
    return false;
  }
  return true;
}

int32_t JetsonVideoDecoder::Decode(const webrtc::EncodedImage& input_image,
                                   bool missing_frames,
                                   int64_t render_time_ms) {
  if (decoder_ == nullptr) {
    return WEBRTC_VIDEO_CODEC_UNINITIALIZED;
  }
  if (decode_complete_callback_ == NULL) {
    return WEBRTC_VIDEO_CODEC_UNINITIALIZED;
  }
  if (input_image.data() == NULL && input_image.size() > 0) {
    return WEBRTC_VIDEO_CODEC_ERR_PARAMETER;
  }

  struct v4l2_buffer v4l2_buf;
  struct v4l2_plane planes[MAX_PLANES];
  NvBuffer* buffer;

  memset(&v4l2_buf, 0, sizeof(v4l2_buf));
  memset(planes, 0, sizeof(planes));
  v4l2_buf.m.planes = planes;

  // RTC_LOG(LS_INFO) << __FUNCTION__ << " output_plane.getNumBuffers: "
  //                  << decoder_->output_plane.getNumBuffers()
  //                  << " output_plane.getNumQueuedBuffers: "
  //                  << decoder_->output_plane.getNumQueuedBuffers();

  if (decoder_->output_plane.getNumQueuedBuffers() ==
      decoder_->output_plane.getNumBuffers()) {
    if (decoder_->output_plane.dqBuffer(v4l2_buf, &buffer, NULL, -1) < 0) {
      RTC_LOG(LS_ERROR) << "Failed to dqBuffer at decoder output_plane";
      return WEBRTC_VIDEO_CODEC_ERROR;
    }
  } else {
    buffer = decoder_->output_plane.getNthBuffer(
        decoder_->output_plane.getNumQueuedBuffers());
    v4l2_buf.index = decoder_->output_plane.getNumQueuedBuffers();
  }

  memcpy(buffer->planes[0].data, input_image.data(), input_image.size());
  buffer->planes[0].bytesused = input_image.size();

  v4l2_buf.m.planes[0].bytesused = buffer->planes[0].bytesused;

  v4l2_buf.flags |= V4L2_BUF_FLAG_TIMESTAMP_COPY;
  v4l2_buf.timestamp.tv_sec =
      input_image.Timestamp() / rtc::kNumMicrosecsPerSec;
  v4l2_buf.timestamp.tv_usec =
      input_image.Timestamp() % rtc::kNumMicrosecsPerSec;

  if (decoder_->output_plane.qBuffer(v4l2_buf, nullptr) < 0) {
    RTC_LOG(LS_ERROR) << "Failed to qBuffer at encoder output_plane";
    return WEBRTC_VIDEO_CODEC_ERROR;
  }

  // RTC_LOG(LS_INFO) << __FUNCTION__ << " timestamp:" << input_image.Timestamp()
  //                  << " bytesused:" << buffer->planes[0].bytesused;
  return WEBRTC_VIDEO_CODEC_OK;
}

int32_t JetsonVideoDecoder::RegisterDecodeCompleteCallback(
    webrtc::DecodedImageCallback* callback) {
  decode_complete_callback_ = callback;
  return WEBRTC_VIDEO_CODEC_OK;
}

int32_t JetsonVideoDecoder::Release() {
  JetsonRelease();
  buffer_pool_.Release();
  return WEBRTC_VIDEO_CODEC_OK;
}

const char* JetsonVideoDecoder::ImplementationName() const {
  return "Jetson Video";
}

int32_t JetsonVideoDecoder::JetsonConfigure() {
  int ret = 0;

  decoder_ = NvVideoDecoder::createVideoDecoder("dec0");
  INIT_ERROR(!decoder_, "Failed to createVideoDecoder");

  ret = decoder_->subscribeEvent(V4L2_EVENT_RESOLUTION_CHANGE, 0, 0);
  INIT_ERROR(ret < 0,
             "Failed to decoder subscribeEvent V4L2_EVENT_RESOLUTION_CHANGE");

  ret = decoder_->setOutputPlaneFormat(input_format_, CHUNK_SIZE);
  INIT_ERROR(ret < 0, "Failed to decoder setOutputPlaneFormat");

  // 入力されるのがフレーム単位でない場合は 1 を設定する
  ret = decoder_->setFrameInputMode(1);
  INIT_ERROR(ret < 0, "Failed to decoder setFrameInputMode");

  ret = decoder_->output_plane.setupPlane(V4L2_MEMORY_MMAP, 10, true, false);
  INIT_ERROR(ret < 0, "Failed to setupPlane at decoder output_plane");

  ret = decoder_->subscribeEvent(V4L2_EVENT_EOS, 0, 0);
  INIT_ERROR(ret < 0, "Failed to subscribeEvent V4L2_EVENT_EOS");

  ret = decoder_->output_plane.setStreamStatus(true);
  INIT_ERROR(ret < 0, "Failed to setStreamStatus at decoder output_plane");

  if (capture_loop_.empty()) {
    eos_ = false;
    capture_loop_ = rtc::PlatformThread::SpawnJoinable(
        std::bind(JetsonVideoDecoder::CaptureLoopFunction, this), "CaptureLoop",
        rtc::ThreadAttributes().SetPriority(rtc::ThreadPriority::kHigh));
  }

  return WEBRTC_VIDEO_CODEC_OK;
}

bool JetsonVideoDecoder::JetsonRelease() {
  if (decoder_) {
    if (!capture_loop_.empty()) {
      eos_ = true;
      SendEOS(decoder_);
      while (decoder_->output_plane.getNumQueuedBuffers() > 0 && !got_error_ &&
             !decoder_->isInError()) {
        struct v4l2_buffer v4l2_buf;
        struct v4l2_plane planes[MAX_PLANES];

        memset(&v4l2_buf, 0, sizeof(v4l2_buf));
        memset(planes, 0, sizeof(planes));

        v4l2_buf.m.planes = planes;
        if (decoder_->output_plane.dqBuffer(v4l2_buf, NULL, NULL, -1) < 0) {
          RTC_LOG(LS_ERROR)
              << __FUNCTION__ << " Failed to dqBuffer at decoder output_plane";
          got_error_ = true;
          break;
        }
      }
      capture_loop_.Finalize();
    }
    delete decoder_;
    decoder_ = nullptr;
  }
  if (dst_dma_fd_ != -1) {
    NvBufSurf::NvDestroy(dst_dma_fd_);
    dst_dma_fd_ = -1;
  }
  return true;
}

void JetsonVideoDecoder::SendEOS(NvV4l2Element* element) {
  if (element->output_plane.getStreamStatus()) {
    struct v4l2_buffer v4l2_buf;
    struct v4l2_plane planes[MAX_PLANES];
    NvBuffer* buffer;

    memset(&v4l2_buf, 0, sizeof(v4l2_buf));
    memset(planes, 0, MAX_PLANES * sizeof(struct v4l2_plane));
    v4l2_buf.m.planes = planes;

    while (element->output_plane.getNumQueuedBuffers() != 0) {
      if (element->output_plane.dqBuffer(v4l2_buf, &buffer, NULL, 10) < 0) {
        RTC_LOG(LS_ERROR) << "Failed to dqBuffer at encoder output_plane";
      }
    }
    planes[0].bytesused = 0;
    if (element->output_plane.qBuffer(v4l2_buf, NULL) < 0) {
      RTC_LOG(LS_ERROR) << "Failed to qBuffer at encoder output_plane";
    }
  }
}

void JetsonVideoDecoder::CaptureLoopFunction(void* obj) {
  JetsonVideoDecoder* _this = static_cast<JetsonVideoDecoder*>(obj);
  _this->CaptureLoop();
}

void JetsonVideoDecoder::CaptureLoop() {
  struct v4l2_event event;
  int ret;
  do {
    ret = decoder_->dqEvent(event, 10);
    if (eos_) {
      return;
    }
    if (ret < 0) {
      if (errno == EAGAIN) {
        continue;
      } else {
        RTC_LOG(LS_ERROR) << __FUNCTION__ << " Failed to dqEvent at decoder";
        got_error_ = true;
        break;
      }
    }
  } while (event.type != V4L2_EVENT_RESOLUTION_CHANGE && !got_error_);

  if (!got_error_) {
    SetCapture();
  }

  while (!(eos_ || got_error_ || decoder_->isInError())) {
    ret = decoder_->dqEvent(event, false);
    if (ret == 0 && event.type == V4L2_EVENT_RESOLUTION_CHANGE) {
      SetCapture();
      continue;
    }

    NvBuffer* buffer;
    while (1) {
      struct v4l2_buffer v4l2_buf;
      struct v4l2_plane planes[MAX_PLANES];

      memset(&v4l2_buf, 0, sizeof(v4l2_buf));
      memset(planes, 0, sizeof(planes));
      v4l2_buf.m.planes = planes;

      // Dequeue a filled buffer
      if (decoder_->capture_plane.dqBuffer(v4l2_buf, &buffer, NULL, 0)) {
        if (errno == EAGAIN) {
          usleep(1000);
        } else {
          RTC_LOG(LS_ERROR)
              << __FUNCTION__ << " Failed to dqBuffer at decoder capture_plane";
          got_error_ = true;
        }
        break;
      }

      uint64_t pts = v4l2_buf.timestamp.tv_sec * rtc::kNumMicrosecsPerSec +
                     v4l2_buf.timestamp.tv_usec;

      NvBufSurf::NvCommonTransformParams transform_params;
      memset(&transform_params, 0, sizeof(transform_params));
      transform_params.src_top = capture_crop_->c.top;
      transform_params.src_left = capture_crop_->c.left;
      transform_params.src_width = capture_crop_->c.width;
      transform_params.src_height = capture_crop_->c.height;
      transform_params.dst_top = 0;
      transform_params.dst_left = 0;
      transform_params.dst_width = capture_crop_->c.width;
      transform_params.dst_height = capture_crop_->c.height;
      transform_params.flag = NVBUFSURF_TRANSFORM_FILTER;
      transform_params.flip = NvBufSurfTransform_None;
      transform_params.filter = NvBufSurfTransformInter_Algo3;
      // 何が来ても YUV420 に変換する
      ret = NvBufSurf::NvTransform(&transform_params, buffer->planes[0].fd,
                                   dst_dma_fd_);
      if (ret == -1) {
        RTC_LOG(LS_ERROR) << __FUNCTION__ << " Transform failed";
        break;
      }

      rtc::scoped_refptr<webrtc::I420Buffer> i420_buffer =
          buffer_pool_.CreateI420Buffer(capture_crop_->c.width,
                                        capture_crop_->c.height);
      if (!i420_buffer.get()) {
        // Pool has too many pending frames.
        RTC_HISTOGRAM_BOOLEAN(
            "WebRTC.Video.LibvpxVp8Decoder.TooManyPendingFrames", 1);
        got_error_ = true;
        break;
      }

      void* src_data;
      uint8_t* dst_data;
      int dst_stride;
      for (uint32_t i = 0; i < MAX_PLANES; i++) {
        if (i == 0) {
          dst_data = i420_buffer->MutableDataY();
          dst_stride = i420_buffer->StrideY();
        } else if (i == 1) {
          dst_data = i420_buffer->MutableDataU();
          dst_stride = i420_buffer->StrideU();
        } else if (i == 2) {
          dst_data = i420_buffer->MutableDataV();
          dst_stride = i420_buffer->StrideV();
        } else {
          break;
        }
        NvBufSurface* dst_surf = 0;

        if (NvBufSurfaceFromFd(dst_dma_fd_, (void**)(&dst_surf)) == -1) {
          RTC_LOG(LS_ERROR) << __FUNCTION__ << "Failed to NvBufSurfaceFromFd";
          break;
        }

        ret = NvBufSurfaceMap(dst_surf, 0, i, NVBUF_MAP_READ);
        NvBufSurfaceSyncForCpu(dst_surf, 0, i);
        src_data = dst_surf->surfaceList[0].mappedAddr.addr[i];

        NvBufSurfacePlaneParams params = dst_surf->surfaceList[0].planeParams;
        for (uint32_t j = 0; j < params.height[i]; j++) {
          memcpy(dst_data + j * dst_stride,
                 (char*)src_data + j * params.pitch[i], params.width[i]);
        }
        NvBufSurfaceUnMap(dst_surf, 0, i);
      }

      webrtc::VideoFrame decoded_image =
          webrtc::VideoFrame::Builder()
              .set_video_frame_buffer(i420_buffer)
              .set_timestamp_rtp(pts)
              .build();
      decode_complete_callback_->Decoded(decoded_image, absl::nullopt,
                                         absl::nullopt);

      if (decoder_->capture_plane.qBuffer(v4l2_buf, NULL) < 0) {
        RTC_LOG(LS_ERROR) << __FUNCTION__
                          << "Failed to qBuffer at capture_plane";
        got_error_ = true;
        break;
      }
    }
  }
}

int JetsonVideoDecoder::SetCapture() {
  int32_t ret;

  struct v4l2_format format;
  ret = decoder_->capture_plane.getFormat(format);
  INIT_ERROR(ret < 0, "Failed to getFormat at capture_plane");

  capture_crop_.reset(new v4l2_crop());
  ret = decoder_->capture_plane.getCrop(*capture_crop_.get());
  INIT_ERROR(ret < 0, "Failed to getCrop at capture_plane");

  RTC_LOG(LS_INFO) << __FUNCTION__ << " plane format "
                   << format.fmt.pix_mp.pixelformat << " "
                   << format.fmt.pix_mp.width << "x"
                   << format.fmt.pix_mp.height;
  RTC_LOG(LS_INFO) << __FUNCTION__ << " crop " << capture_crop_->c.top << "x"
                   << capture_crop_->c.left << " " << capture_crop_->c.width
                   << "x" << format.fmt.pix_mp.height;

  if (dst_dma_fd_ != -1) {
    NvBufSurf::NvDestroy(dst_dma_fd_);
    dst_dma_fd_ = -1;
  }

  NvBufSurf::NvCommonAllocateParams cParams;
  cParams.width = capture_crop_->c.width;
  cParams.height = capture_crop_->c.height;
  cParams.layout = NVBUF_LAYOUT_PITCH;
  cParams.colorFormat = NVBUF_COLOR_FORMAT_YUV420;
  cParams.memtag = NvBufSurfaceTag_VIDEO_DEC;
  cParams.memType = NVBUF_MEM_SURFACE_ARRAY;

  ret = NvBufSurf::NvAllocate(&cParams, 1, &dst_dma_fd_);
  INIT_ERROR(ret == -1, "failed to NvBufSurfaceAllocate");

  decoder_->capture_plane.deinitPlane();

  ret = decoder_->setCapturePlaneFormat(format.fmt.pix_mp.pixelformat,
                                        format.fmt.pix_mp.width,
                                        format.fmt.pix_mp.height);
  INIT_ERROR(ret < 0, "Failed to setCapturePlaneFormat at capture_plane");

  int32_t min_capture_buffer_size;
  ret = decoder_->getMinimumCapturePlaneBuffers(min_capture_buffer_size);
  INIT_ERROR(ret < 0, "Failed to getMinimumCapturePlaneBuffers");

  ret = decoder_->capture_plane.setupPlane(
      V4L2_MEMORY_MMAP, min_capture_buffer_size + 5, false, false);
  INIT_ERROR(ret < 0, "Failed to setupPlane at capture_plane");

  ret = decoder_->capture_plane.setStreamStatus(true);
  INIT_ERROR(ret < 0, "Failed to setStreamStatus at decoder capture_plane");

  for (uint32_t i = 0; i < decoder_->capture_plane.getNumBuffers(); i++) {
    struct v4l2_buffer v4l2_buf;
    struct v4l2_plane planes[MAX_PLANES];
    memset(&v4l2_buf, 0, sizeof(v4l2_buf));
    memset(planes, 0, MAX_PLANES * sizeof(struct v4l2_plane));
    v4l2_buf.index = i;
    v4l2_buf.m.planes = planes;
    v4l2_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    v4l2_buf.memory = V4L2_MEMORY_MMAP;
    ret = decoder_->capture_plane.qBuffer(v4l2_buf, NULL);
    INIT_ERROR(ret < 0, "Failed to qBuffer at encoder capture_plane");
  }

  return WEBRTC_VIDEO_CODEC_OK;
}
