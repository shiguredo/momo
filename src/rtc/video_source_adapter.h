#ifndef VIDEO_SOURCE_ADAPTER_H_
#define VIDEO_SOURCE_ADAPTER_H_

#include "api/video/video_frame.h"
#include "api/video/video_sink_interface.h"
#include "rtc_base/criticalsection.h"

#include "scalable_track_source.h"

class VideoSourceAdapter : public rtc::VideoSinkInterface<webrtc::VideoFrame>
{
private:
  ScalableVideoTrackSource* _video_track_source;
  rtc::CriticalSection _critSect;

public:
  VideoSourceAdapter() : _video_track_source(nullptr)
  {
  }

  // rtc::VideoSinkInterface interface.
  void OnFrame(const webrtc::VideoFrame &frame) override
  {
    rtc::CritScope lock(&_critSect);
    if (_video_track_source) {
      _video_track_source->OnCapturedFrame(frame);
    }
  }

  void SetVideoTrackSource(ScalableVideoTrackSource* video_track_source)
  {
    rtc::CritScope lock(&_critSect);
    _video_track_source = video_track_source;
  }
};

#endif