#ifndef DATA_MANAGER_H_
#define DATA_MANAGER_H_

#include "api/data_channel_interface.h"

class RTCDataManager {
 public:
  virtual ~RTCDataManager() = default;
  // DataChannel は 必ず kClosed にステートが遷移するので OnRemove は不要
  virtual void OnDataChannel(
      rtc::scoped_refptr<webrtc::DataChannelInterface> data_channel) = 0;
};

#endif