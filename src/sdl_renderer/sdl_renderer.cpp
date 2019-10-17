#include "sdl_renderer.h"

#include <cmath>

#include "api/video/i420_buffer.h"
#include "rtc_base/logging.h"
#include "third_party/libyuv/include/libyuv/video_common.h"
#include "third_party/libyuv/include/libyuv/convert_from.h"


#define STD_ASPECT 1.33
#define WIDE_ASPECT 1.78
#define FRAME_INTERVAL (1000 / 30)

SDLRenderer::SDLRenderer() 
      : running_(true),
        window_(nullptr),
        renderer_(nullptr),
        dispatch_(nullptr),
        width_(640), height_(480),
        rows_(1), cols_(1) {
  window_aspect_ = (float)width_ / (float)height_;
  is_wide_ = window_aspect_ > ((STD_ASPECT + WIDE_ASPECT) / 2.0);

  if (SDL_Init(SDL_INIT_VIDEO) < 0) {
    RTC_LOG(LS_ERROR) << __FUNCTION__ << ": SDL_Init failed " << SDL_GetError();
    return;
  }

  window_ = SDL_CreateWindow("Momo WebRTC Native Client",
                            SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
                            width_, height_, SDL_WINDOW_OPENGL );
  if (window_ == nullptr) {
    RTC_LOG(LS_ERROR) << __FUNCTION__ << ": SDL_CreateWindow failed " << SDL_GetError();
    return;
  }

  thread_ = SDL_CreateThread(SDLRenderer::RenderThreadExec, "Render", this);
}

SDLRenderer::~SDLRenderer() {
  running_ = false;
  int ret = 0;
  SDL_WaitThread(thread_, &ret);
  if (ret != 0) {
    RTC_LOG(LS_ERROR) << __FUNCTION__ << ": SDL Thread error:" << ret;
  }
  if (window_) {
    SDL_DestroyWindow(window_);
  }
  SDL_Quit();
}

void SDLRenderer::PollEvent() {
  SDL_Event e;
  // 必ずメインスレッドから呼び出す
  SDL_PollEvent(&e);
}

void SDLRenderer::SetDispatchFunction(
    std::function<void (std::function<void ()>)> dispatch) {
  rtc::CritScope lock(&sinks_lock_);
  dispatch_ = std::move(dispatch);
}

int SDLRenderer::RenderThreadExec(void *data) {
  return ((SDLRenderer *)data)->RenderThread();
}

int SDLRenderer::RenderThread() {
  renderer_ = SDL_CreateRenderer(window_, -1, SDL_RENDERER_ACCELERATED);
  if (renderer_ == nullptr) {
    RTC_LOG(LS_ERROR) << __FUNCTION__ << ": SDL_CreateRenderer failed " << SDL_GetError();
    return 1;
  }
  SDL_SetRenderDrawColor(renderer_, 0, 0, 0, 255);

  uint32_t start_time, duration;
  while (running_) {
    start_time = SDL_GetTicks();
    {
      rtc::CritScope lock(&sinks_lock_);
      SDL_RenderClear(renderer_);
      for (const VideoTrackSinkVector::value_type& sinks : sinks_) {
        Sink *sink = sinks.second.get();

        rtc::CritScope frame_lock(sink->GetCriticalSection());

        if (!sink->GetOutlineChanged()) continue;

        int width = sink->GetWidth();
        int height = sink->GetHeight();

        if (width == 0 || height == 0) continue;

        SDL_Surface *surface = SDL_CreateRGBSurfaceFrom(
            sink->GetImage(), width, height, 32, width * 4, 0, 0, 0, 0);
        SDL_Texture *texture = SDL_CreateTextureFromSurface(renderer_, surface);
        SDL_FreeSurface(surface);

        SDL_Rect image_rect = (SDL_Rect){ 0, 0, width, height};
        SDL_Rect draw_rect = (SDL_Rect){ sink->GetOffsetX(), sink->GetOffsetY(), width, height};

        // flip (自画像とか？)
        //SDL_RenderCopyEx(renderer_, texture, &image_rect, &draw_rect, 0, nullptr, SDL_FLIP_HORIZONTAL);
        SDL_RenderCopy(renderer_, texture, &image_rect, &draw_rect);

        SDL_DestroyTexture(texture);
      }
      SDL_RenderPresent(renderer_);
      
      if (dispatch_) {
        dispatch_(std::bind(&SDLRenderer::PollEvent, this));
      }
    }
    duration = SDL_GetTicks() - start_time;
    SDL_Delay(FRAME_INTERVAL - (duration % FRAME_INTERVAL));
  }

  SDL_DestroyRenderer(renderer_);

  return 0;
}

SDLRenderer::Sink::Sink(SDLRenderer* renderer,
                         webrtc::VideoTrackInterface* track)
                         : renderer_(renderer),
                           track_(track),
                           outline_offset_x_(0),
                           outline_offset_y_(0),
                           outline_width_(0),
                           outline_height_(0),
                           outline_changed_(false),
                           input_width_(0),
                           input_height_(0),
                           width_(0),
                           height_(0) {
  track_->AddOrUpdateSink(this, rtc::VideoSinkWants());
}

SDLRenderer::Sink::~Sink() {
  track_->RemoveSink(this);
}

void SDLRenderer::Sink::OnFrame(const webrtc::VideoFrame& frame) {
  if (outline_width_ == 0 || outline_height_ == 0) return;
  if (frame.width() == 0 || frame.height() == 0) return;
  rtc::CritScope lock(GetCriticalSection());
  if (outline_changed_ || frame.width() != input_width_ || frame.height() != input_height_) {
    int width, height;
    float frame_aspect = (float)frame.width() / (float)frame.height();
    if (frame_aspect > outline_aspect_) {
      width = outline_width_;
      height = width / frame_aspect;
      offset_y_ = (outline_height_ - height) / 2;
    } else {
      height = outline_height_;
      width = height * frame_aspect;
      offset_x_ = (outline_width_ - width) / 2;
    }
    if (width_ != width || height_ != height) {
      width_ = width;
      height_ = height;
      image_.reset(new uint8_t[width_ * height_ * 4]);
    }
    input_width_ = frame.width();
    input_height_ = frame.height();
    outline_changed_ = false;
  }
  rtc::scoped_refptr<webrtc::I420Buffer> buffer =
      webrtc::I420Buffer::Create(width_, height_);
  buffer->ScaleFrom(*frame.video_frame_buffer()->ToI420());
  if (frame.rotation() != webrtc::kVideoRotation_0) {
    buffer = webrtc::I420Buffer::Rotate(*buffer, frame.rotation());
  }
  libyuv::ConvertFromI420(
      buffer->DataY(), buffer->StrideY(),
      buffer->DataU(), buffer->StrideU(),
      buffer->DataV(), buffer->StrideV(),
      image_.get(), width_ * 4,
      buffer->width(), buffer->height(),
      libyuv::FOURCC_ARGB);
}

void SDLRenderer::Sink::SetOutlineRect(int x, int y, int width, int height) {
  outline_offset_x_ = x;
  outline_offset_y_ = y;
  if (outline_width_ == width && outline_height_ == height) {
    return;
  }
  rtc::CritScope lock(GetCriticalSection());
  offset_y_ = 0;
  offset_x_ = 0;
  outline_width_ = width;
  outline_height_ = height;
  outline_aspect_ = (float)outline_width_ / (float)outline_height_;
  outline_changed_ = true;
}

rtc::CriticalSection* SDLRenderer::Sink::GetCriticalSection() {
  return &frame_params_lock_;
}

bool SDLRenderer::Sink::GetOutlineChanged() {
  return !outline_changed_;
}

int SDLRenderer::Sink::GetOffsetX() {
  return outline_offset_x_ + offset_x_;
}

int SDLRenderer::Sink::GetOffsetY() {
  return outline_offset_y_ + offset_y_;
}

int SDLRenderer::Sink::GetWidth() {
  return width_;
}

int SDLRenderer::Sink::GetHeight() {
  return height_;
}

uint8_t* SDLRenderer::Sink::GetImage() {
  return image_.get();
}

void SDLRenderer::SetOutlines() {
  float frame_aspect = is_wide_ ? WIDE_ASPECT : STD_ASPECT;
  int rows = 1;
  int cols = 1;
  if (window_aspect_ >= 1.0) {
    int times = std::floor(window_aspect_ / frame_aspect);
    if (times < 1) times = 1;
    while (rows * cols < sinks_.size()) {
      if (times < (cols / rows))
      {
        rows++;
      } else {
        cols++;
      }
    }
  } else {
    int times = std::floor(frame_aspect / window_aspect_);
    if (times < 1) times = 1;
    while (rows * cols < sinks_.size()) {
      if (times < (rows / cols))
      {
        cols++;
      } else {
        rows++;
      }
    }
  }
  RTC_LOG(LS_VERBOSE) << __FUNCTION__
                    << " rows:" << rows
                    << " cols:" << cols;
  int outline_width = std::floor(width_ / cols);
  int outline_height = std::floor(height_ / rows);
  int sinks_count = sinks_.size();
  for (int i = 0; i < sinks_count; i++) {
    Sink *sink = sinks_[i].second.get();
    int offset_x = outline_width * (i % cols);
    int offset_y = outline_height * std::floor(i / cols);
    sink->SetOutlineRect(
          offset_x, offset_y, outline_width, outline_height);
    RTC_LOG(LS_VERBOSE) << __FUNCTION__
                      << " offset_x:" << offset_x
                      << " offset_y:" << offset_y
                      << " outline_width:" << outline_width
                      << " outline_height:" << outline_height;
  }
  rows_ = rows;
  cols_ = cols;
}

void SDLRenderer::AddTrack(webrtc::VideoTrackInterface* track) {
  std::unique_ptr<Sink> sink(new Sink(this, track));
  rtc::CritScope lock(&sinks_lock_);
  sinks_.push_back(std::make_pair(track, std::move(sink)));
  SetOutlines();
}

void SDLRenderer::RemoveTrack(webrtc::VideoTrackInterface* track) {
  rtc::CritScope lock(&sinks_lock_);
  sinks_.erase(
      std::remove_if(
          sinks_.begin(), sinks_.end(),
          [track](const VideoTrackSinkVector::value_type& sink) {
            return sink.first == track;
          }),
      sinks_.end());
  SetOutlines();
}