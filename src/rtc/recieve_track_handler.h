#ifndef RTC_RECIEVE_TRACK_HANDLER_H_
#define RTC_RECIEVE_TRACK_HANDLER_H_

#include <string>
#include "api/video/video_sink_interface.h"

class RTCRecieveTrackHandler {
 public:
  virtual void AddTrack(webrtc::VideoTrackInterface* track) = 0;
  virtual void RemoveTrack(webrtc::VideoTrackInterface* track) = 0;
};

#endif