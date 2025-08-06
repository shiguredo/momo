#include "rtc/fake_video_capturer.h"

#if defined(USE_FAKE_CAPTURE_DEVICE)

#include <cstring>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <sstream>

// WebRTC
#include <rtc_base/logging.h>
#include <third_party/libyuv/include/libyuv.h>

FakeVideoCapturer::FakeVideoCapturer(Config config)
    : sora::ScalableVideoTrackSource(config), config_(config) {
  StartCapture();
}

FakeVideoCapturer::~FakeVideoCapturer() {
  StopCapture();
}

void FakeVideoCapturer::StartCapture() {
  if (capture_thread_) {
    return;
  }

  stop_capture_ = false;
  frame_counter_ = 0;
  start_time_ = std::chrono::high_resolution_clock::now();
  
  capture_thread_ = std::make_unique<std::thread>([this] { CaptureThread(); });
}

void FakeVideoCapturer::StopCapture() {
  if (!capture_thread_) {
    return;
  }

  stop_capture_ = true;
  if (capture_thread_->joinable()) {
    capture_thread_->join();
  }
  capture_thread_.reset();
}

void FakeVideoCapturer::CaptureThread() {
  // Blend2D イメージとフォントの初期化
  image_.create(config_.width, config_.height, BL_FORMAT_PRGB32);
  frame_counter_ = 0;
  
  // フォントの作成（現時点ではフォントなしで動作）
  // TODO: フォントファイルを埋め込んで対応
  has_font_ = false;
  
  while (!stop_capture_) {
    auto now = std::chrono::high_resolution_clock::now();
    
    // 画像を更新
    UpdateImage(now);
    
    // Blend2D イメージから I420 バッファへ変換
    BLImageData data;
    BLResult result = image_.getData(&data);
    if (result != BL_SUCCESS) {
      std::this_thread::sleep_for(std::chrono::milliseconds(16));
      continue;
    }
    
    webrtc::scoped_refptr<webrtc::I420Buffer> buffer = 
        webrtc::I420Buffer::Create(config_.width, config_.height);
    
    libyuv::ABGRToI420(
        (const uint8_t*)data.pixelData, data.stride,
        buffer->MutableDataY(), buffer->StrideY(),
        buffer->MutableDataU(), buffer->StrideU(),
        buffer->MutableDataV(), buffer->StrideV(),
        config_.width, config_.height);
    
    // タイムスタンプを計算
    int64_t timestamp_us = 
        std::chrono::duration_cast<std::chrono::microseconds>(
            now - start_time_).count();
    
    // フレームを送信
    bool captured = OnCapturedFrame(
        webrtc::VideoFrame::Builder()
            .set_video_frame_buffer(buffer)
            .set_rotation(webrtc::kVideoRotation_0)
            .set_timestamp_us(timestamp_us)
            .build());
    
    if (captured) {
      std::this_thread::sleep_for(
          std::chrono::milliseconds(1000 / config_.fps - 2));
      frame_counter_++;
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
}

void FakeVideoCapturer::UpdateImage(
    std::chrono::high_resolution_clock::time_point now) {
  BLContext ctx(image_);
  
  ctx.setCompOp(BL_COMP_OP_SRC_COPY);
  ctx.fillAll();
  
  ctx.save();
  DrawTexts(ctx, now);
  ctx.restore();
  
  ctx.save();
  DrawAnimations(ctx, now);
  ctx.restore();
  
  ctx.save();
  DrawBoxes(ctx, now);
  ctx.restore();
  
  ctx.end();
}

void FakeVideoCapturer::DrawTexts(
    BLContext& ctx,
    std::chrono::high_resolution_clock::time_point now) {
  // フォントがない場合はテキスト描画をスキップ
  if (!has_font_) {
    return;
  }
  
  auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      now - start_time_).count();
  
  ctx.setFillStyle(BLRgba32(0xFFFFFFFF));
  
  int width = config_.width;
  int height = config_.height;
  int fps = config_.fps;
  
  // タイマー表示
  {
    int hours = ms / (60 * 60 * 1000);
    int minutes = (ms / (60 * 1000)) % 60;
    int seconds = (ms / 1000) % 60;
    int milliseconds = ms % 1000;
    
    std::stringstream ss;
    ss << std::setfill('0') << std::setw(2) << hours << ":"
       << std::setfill('0') << std::setw(2) << minutes << ":"
       << std::setfill('0') << std::setw(2) << seconds << "."
       << std::setfill('0') << std::setw(3) << milliseconds;
    
    ctx.fillUtf8Text(BLPoint(width * 0.05, height * 0.15), 
                     base_font_, ss.str().c_str());
  }
  
  // フレーム番号表示
  {
    std::stringstream ss;
    ss << std::setfill('0') << std::setw(6) << frame_counter_;
    ctx.fillUtf8Text(BLPoint(width * 0.05, height * 0.15 + base_font_.size()),
                     base_font_, ss.str().c_str());
  }
  
  // FPS とサイズ情報
  {
    std::string text = "Requested frame rate: " + std::to_string(fps) + " fps";
    ctx.fillUtf8Text(BLPoint(width * 0.45, height * 0.75), 
                     stats_font_, text.c_str());
  }
  {
    std::string text = "Size: " + std::to_string(width) + " x " + 
                      std::to_string(height);
    ctx.fillUtf8Text(BLPoint(width * 0.45, height * 0.75 + stats_font_.size()),
                     stats_font_, text.c_str());
  }
  
  // Bip/Bop アニメーション
  {
    int m = frame_counter_ % 60;
    if (m < 15) {
      ctx.setFillStyle(BLRgba32(0, 255, 255));
      ctx.fillUtf8Text(BLPoint(width * 0.6, height * 0.6), 
                       bipbop_font_, "Bip");
    } else if (m >= 30 && m < 45) {
      ctx.setFillStyle(BLRgba32(255, 255, 0));
      ctx.fillUtf8Text(BLPoint(width * 0.6, height * 0.6), 
                       bipbop_font_, "Bop");
    }
  }
}

void FakeVideoCapturer::DrawAnimations(
    BLContext& ctx,
    std::chrono::high_resolution_clock::time_point now) {
  int width = config_.width;
  int height = config_.height;
  int fps = config_.fps;
  
  const float pi = 3.14159f;
  ctx.translate(width * 0.8, height * 0.3);
  ctx.rotate(-pi / 2);
  ctx.setFillStyle(BLRgba32(255, 255, 255));
  ctx.fillPie(0, 0, width * 0.09, 0, 2 * pi);
  
  ctx.setFillStyle(BLRgba32(160, 160, 160));
  ctx.fillPie(0, 0, width * 0.09, 0, 
              (frame_counter_ % fps) / static_cast<float>(fps) * 2 * pi);
}

void FakeVideoCapturer::DrawBoxes(
    BLContext& ctx,
    std::chrono::high_resolution_clock::time_point now) {
  int width = config_.width;
  int height = config_.height;
  
  // 移動するボックスのアニメーション
  const int box_size = 50;
  const int num_boxes = 5;
  
  for (int i = 0; i < num_boxes; i++) {
    double phase = (frame_counter_ + i * 20) % 100 / 100.0;
    double x = phase * (width - box_size);
    double y = height * 0.5 + sin(phase * 3.14159 * 2) * height * 0.2;
    
    // 各ボックスに異なる色を設定
    uint32_t color = 0xFF000000;
    switch (i % 5) {
      case 0: color |= 0xFF0000; break;  // 赤
      case 1: color |= 0x00FF00; break;  // 緑
      case 2: color |= 0x0000FF; break;  // 青
      case 3: color |= 0xFFFF00; break;  // 黄
      case 4: color |= 0xFF00FF; break;  // マゼンタ
    }
    
    ctx.setFillStyle(BLRgba32(color));
    ctx.fillRect(x, y, box_size, box_size);
  }
}

#endif  // USE_FAKE_CAPTURE_DEVICE