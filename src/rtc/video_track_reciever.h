#ifndef VIDEO_TRACK_RECIEVER_HANDLER_H_
#define VIDEO_TRACK_RECIEVER_HANDLER_H_

#include <string>
#include "api/video/video_sink_interface.h"

class VideoTrackReciever {
 public:
  virtual void AddTrack(webrtc::VideoTrackInterface* track) = 0;
  virtual void RemoveTrack(webrtc::VideoTrackInterface* track) = 0;
};

#endif