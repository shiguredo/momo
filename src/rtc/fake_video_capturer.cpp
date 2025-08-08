#include "rtc/fake_video_capturer.h"
#include "rtc/fake_audio_capturer.h"

#if defined(USE_FAKE_CAPTURE_DEVICE)

#include <chrono>
#include <cstring>
#include <iomanip>
#include <sstream>

// WebRTC
#include <rtc_base/logging.h>
#include <third_party/libyuv/include/libyuv.h>

FakeVideoCapturer::FakeVideoCapturer(
    Config config,
    webrtc::scoped_refptr<FakeAudioCapturer> audio_capturer)
    : sora::ScalableVideoTrackSource(config),
      config_(config),
      audio_capturer_(audio_capturer) {
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
  // Blend2D イメージの初期化
  image_.create(config_.width, config_.height, BL_FORMAT_PRGB32);
  frame_counter_ = 0;

  while (!stop_capture_) {
    auto now = std::chrono::high_resolution_clock::now();

    // 画像を更新
    UpdateImage(now);

    // Blend2D イメージから I420 バッファへ変換
    BLImageData data;
    BLResult result = image_.getData(&data);
    if (result != BL_SUCCESS) {
      RTC_LOG(LS_ERROR) << "Failed to get image data from Blend2D: " << result;
      std::this_thread::sleep_for(std::chrono::milliseconds(16));
      continue;
    }

    webrtc::scoped_refptr<webrtc::I420Buffer> buffer =
        webrtc::I420Buffer::Create(config_.width, config_.height);

    libyuv::ABGRToI420((const uint8_t*)data.pixelData, data.stride,
                       buffer->MutableDataY(), buffer->StrideY(),
                       buffer->MutableDataU(), buffer->StrideU(),
                       buffer->MutableDataV(), buffer->StrideV(), config_.width,
                       config_.height);

    // タイムスタンプを計算
    int64_t timestamp_us =
        std::chrono::duration_cast<std::chrono::microseconds>(now - start_time_)
            .count();

    // フレームを送信
    bool captured = OnCapturedFrame(webrtc::VideoFrame::Builder()
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

  // デジタル時計を描画
  ctx.save();
  DrawDigitalClock(ctx, now);
  ctx.restore();

  ctx.save();
  DrawAnimations(ctx, now);
  ctx.restore();

  ctx.save();
  DrawBoxes(ctx, now);
  ctx.restore();

  ctx.end();
}

void FakeVideoCapturer::DrawAnimations(
    BLContext& ctx,
    std::chrono::high_resolution_clock::time_point now) {
  int width = config_.width;
  int height = config_.height;
  int fps = config_.fps;

  const float pi = 3.14159f;
  ctx.translate(width * 0.5, height * 0.5);  // 画面中央に配置
  ctx.rotate(-pi / 2);
  ctx.setFillStyle(BLRgba32(255, 255, 255));
  ctx.fillPie(0, 0, width * 0.3, 0, 2 * pi);  // 大きくする

  ctx.setFillStyle(BLRgba32(160, 160, 160));
  ctx.fillPie(0, 0, width * 0.3, 0,
              (frame_counter_ % fps) / static_cast<float>(fps) * 2 * pi);

  // 円が一周したときにビープ音を鳴らす（0度の位置を通過したとき）
  if (audio_capturer_) {
    // 前フレームと現在フレームの角度を計算
    uint32_t prev_frame = (frame_counter_ > 0) ? frame_counter_ - 1 : fps - 1;
    float prev_angle = (prev_frame % fps) / static_cast<float>(fps) * 360.0f;
    float curr_angle =
        (frame_counter_ % fps) / static_cast<float>(fps) * 360.0f;

    // 0度を通過したかチェック（359度から0度への遷移）
    if (prev_angle > 270.0f && curr_angle < 90.0f) {
      audio_capturer_->TriggerBeep();
    }
  }
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
      case 0:
        color |= 0xFF0000;
        break;  // 赤
      case 1:
        color |= 0x00FF00;
        break;  // 緑
      case 2:
        color |= 0x0000FF;
        break;  // 青
      case 3:
        color |= 0xFFFF00;
        break;  // 黄
      case 4:
        color |= 0xFF00FF;
        break;  // マゼンタ
    }

    ctx.setFillStyle(BLRgba32(color));
    ctx.fillRect(x, y, box_size, box_size);
  }
}

void FakeVideoCapturer::DrawDigitalClock(
    BLContext& ctx,
    std::chrono::high_resolution_clock::time_point now) {
  auto ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time_)
          .count();

  int hours = (ms / (60 * 60 * 1000)) % 10000;  // 9999まで表示
  int minutes = (ms / (60 * 1000)) % 60;
  int seconds = (ms / 1000) % 60;
  int milliseconds = ms % 1000;

  // デジタル時計の配置パラメータ
  double clock_x = config_.width * 0.02;        // 左端に寄せる
  double clock_y = config_.height * 0.02;       // 上端に寄せる
  double digit_width = config_.width * 0.018;   // もっと小さく
  double digit_height = config_.height * 0.04;  // 高さも小さく
  double spacing = digit_width * 0.3;
  double colon_width = digit_width * 0.3;

  ctx.setFillStyle(BLRgba32(0, 255, 255));  // シアン色

  // HHHH:MM:SS.mmm の表示
  double x = clock_x;

  // 時間（4桁）
  Draw7Segment(ctx, (hours / 1000) % 10, x, clock_y, digit_width, digit_height);
  x += digit_width + spacing;
  Draw7Segment(ctx, (hours / 100) % 10, x, clock_y, digit_width, digit_height);
  x += digit_width + spacing;
  Draw7Segment(ctx, (hours / 10) % 10, x, clock_y, digit_width, digit_height);
  x += digit_width + spacing;
  Draw7Segment(ctx, hours % 10, x, clock_y, digit_width, digit_height);
  x += digit_width + spacing;

  // コロン
  DrawColon(ctx, x, clock_y, digit_height);
  x += colon_width + spacing;

  // 分（2桁）
  Draw7Segment(ctx, minutes / 10, x, clock_y, digit_width, digit_height);
  x += digit_width + spacing;
  Draw7Segment(ctx, minutes % 10, x, clock_y, digit_width, digit_height);
  x += digit_width + spacing;

  // コロン
  DrawColon(ctx, x, clock_y, digit_height);
  x += colon_width + spacing;

  // 秒（2桁）
  Draw7Segment(ctx, seconds / 10, x, clock_y, digit_width, digit_height);
  x += digit_width + spacing;
  Draw7Segment(ctx, seconds % 10, x, clock_y, digit_width, digit_height);
  x += digit_width + spacing;

  // ドット
  ctx.fillCircle(x + colon_width * 0.3, clock_y + digit_height * 0.8,
                 digit_height * 0.05);
  x += colon_width + spacing;

  // ミリ秒（3桁、少し小さめに表示）
  double ms_digit_width = digit_width * 0.7;
  double ms_digit_height = digit_height * 0.7;

  ctx.setFillStyle(BLRgba32(200, 200, 200));  // グレー色
  Draw7Segment(ctx, (milliseconds / 100) % 10, x,
               clock_y + (digit_height - ms_digit_height) / 2, ms_digit_width,
               ms_digit_height);
  x += ms_digit_width + spacing * 0.8;
  Draw7Segment(ctx, (milliseconds / 10) % 10, x,
               clock_y + (digit_height - ms_digit_height) / 2, ms_digit_width,
               ms_digit_height);
  x += ms_digit_width + spacing * 0.8;
  Draw7Segment(ctx, milliseconds % 10, x,
               clock_y + (digit_height - ms_digit_height) / 2, ms_digit_width,
               ms_digit_height);
}

void FakeVideoCapturer::Draw7Segment(BLContext& ctx,
                                     int digit,
                                     double x,
                                     double y,
                                     double width,
                                     double height) {
  // 7セグメントディスプレイのセグメント定義
  //  aaa
  // f   b
  //  ggg
  // e   c
  //  ddd

  double thickness = width * 0.15;
  double gap = thickness * 0.2;

  // 各セグメントのON/OFF（0-9の数字に対応）
  bool segments[10][7] = {
      {true, true, true, true, true, true, false},      // 0
      {false, true, true, false, false, false, false},  // 1
      {true, true, false, true, true, false, true},     // 2
      {true, true, true, true, false, false, true},     // 3
      {false, true, true, false, false, true, true},    // 4
      {true, false, true, true, false, true, true},     // 5
      {true, false, true, true, true, true, true},      // 6
      {true, true, true, false, false, false, false},   // 7
      {true, true, true, true, true, true, true},       // 8
      {true, true, true, true, false, true, true}       // 9
  };

  if (digit < 0 || digit > 9)
    return;

  // 横セグメント（a, g, d）
  auto drawHorizontalSegment = [&](double sx, double sy) {
    BLPath path;
    path.moveTo(sx + gap, sy);
    path.lineTo(sx + width - gap, sy);
    path.lineTo(sx + width - gap - thickness * 0.5, sy + thickness * 0.5);
    path.lineTo(sx + width - gap, sy + thickness);
    path.lineTo(sx + gap, sy + thickness);
    path.lineTo(sx + gap + thickness * 0.5, sy + thickness * 0.5);
    path.close();
    ctx.fillPath(path);
  };

  // 縦セグメント（f, b, e, c）
  auto drawVerticalSegment = [&](double sx, double sy, double sh) {
    BLPath path;
    path.moveTo(sx, sy + gap);
    path.lineTo(sx + thickness * 0.5, sy + gap + thickness * 0.5);
    path.lineTo(sx + thickness, sy + gap);
    path.lineTo(sx + thickness, sy + sh - gap);
    path.lineTo(sx + thickness * 0.5, sy + sh - gap - thickness * 0.5);
    path.lineTo(sx, sy + sh - gap);
    path.close();
    ctx.fillPath(path);
  };

  // セグメントa（上）
  if (segments[digit][0]) {
    drawHorizontalSegment(x, y);
  }

  // セグメントb（右上）
  if (segments[digit][1]) {
    drawVerticalSegment(x + width - thickness, y, height * 0.5);
  }

  // セグメントc（右下）
  if (segments[digit][2]) {
    drawVerticalSegment(x + width - thickness, y + height * 0.5, height * 0.5);
  }

  // セグメントd（下）
  if (segments[digit][3]) {
    drawHorizontalSegment(x, y + height - thickness);
  }

  // セグメントe（左下）
  if (segments[digit][4]) {
    drawVerticalSegment(x, y + height * 0.5, height * 0.5);
  }

  // セグメントf（左上）
  if (segments[digit][5]) {
    drawVerticalSegment(x, y, height * 0.5);
  }

  // セグメントg（中央）
  if (segments[digit][6]) {
    drawHorizontalSegment(x, y + height * 0.5 - thickness * 0.5);
  }
}

void FakeVideoCapturer::DrawColon(BLContext& ctx,
                                  double x,
                                  double y,
                                  double height) {
  double dot_size = height * 0.1;
  ctx.fillCircle(x + dot_size, y + height * 0.3, dot_size);
  ctx.fillCircle(x + dot_size, y + height * 0.7, dot_size);
}

#endif  // USE_FAKE_CAPTURE_DEVICE