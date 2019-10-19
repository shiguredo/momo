#ifndef VIDEO_TRACK_RECEIVER_HANDLER_H_
#define VIDEO_TRACK_RECEIVER_HANDLER_H_

#include <string>
#include "api/media_stream_interface.h"

class VideoTrackReceiver {
 public:
  virtual void AddTrack(webrtc::VideoTrackInterface* track) = 0;
  virtual void RemoveTrack(webrtc::VideoTrackInterface* track) = 0;
};

#endif