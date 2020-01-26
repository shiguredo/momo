#ifndef SDL_RENDERER_H_
#define SDL_RENDERER_H_

#include <SDL.h>

#include <boost/asio.hpp>
#include <memory>
#include <string>
#include <vector>

#include "api/media_stream_interface.h"
#include "api/scoped_refptr.h"
#include "api/video/video_frame.h"
#include "api/video/video_sink_interface.h"
#include "rtc/video_track_receiver.h"
#include "rtc_base/critical_section.h"

class SDLRenderer : public VideoTrackReceiver {
 public:
  SDLRenderer(int width, int height, bool fullscreen);
  ~SDLRenderer();

  void SetDispatchFunction(std::function<void(std::function<void()>)> dispatch);

  static int RenderThreadExec(void* data);
  int RenderThread();

  void SetOutlines();
  void AddTrack(webrtc::VideoTrackInterface* track) override;
  void RemoveTrack(webrtc::VideoTrackInterface* track) override;

 protected:
  class Sink : public rtc::VideoSinkInterface<webrtc::VideoFrame> {
   public:
    Sink(SDLRenderer* renderer, webrtc::VideoTrackInterface* track);
    ~Sink();

    void OnFrame(const webrtc::VideoFrame& frame) override;

    void SetOutlineRect(int x, int y, int width, int height);

    rtc::CriticalSection* GetCriticalSection();
    bool GetOutlineChanged();
    int GetOffsetX();
    int GetOffsetY();
    int GetFrameWidth();
    int GetFrameHeight();
    int GetWidth();
    int GetHeight();
    uint8_t* GetImage();

   private:
    SDLRenderer* renderer_;
    rtc::scoped_refptr<webrtc::VideoTrackInterface> track_;
    rtc::CriticalSection frame_params_lock_;
    int outline_offset_x_;
    int outline_offset_y_;
    int outline_width_;
    int outline_height_;
    bool outline_changed_;
    float outline_aspect_;
    int input_width_;
    int input_height_;
    bool scaled_;
    std::unique_ptr<uint8_t[]> image_;
    int offset_x_;
    int offset_y_;
    int width_;
    int height_;
  };

 private:
  bool IsFullScreen();
  void SetFullScreen(bool fullscreen);
  void PollEvent();

  rtc::CriticalSection sinks_lock_;
  typedef std::vector<
      std::pair<webrtc::VideoTrackInterface*, std::unique_ptr<Sink> > >
      VideoTrackSinkVector;
  VideoTrackSinkVector sinks_;
  std::atomic<bool> running_;
  SDL_Thread* thread_;
  SDL_Window* window_;
  SDL_Renderer* renderer_;
  std::function<void(std::function<void()>)> dispatch_;
  int width_;
  int height_;
  int rows_;
  int cols_;
};

#endif