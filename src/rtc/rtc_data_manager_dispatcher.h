#ifndef RTC_DATA_MANAGER_DISPATCHER_H_
#define RTC_DATA_MANAGER_DISPATCHER_H_

#include <vector>

#include "rtc_data_manager.h"

class RTCDataManagerDispatcher : public RTCDataManager {
 public:
  void Add(RTCDataManager* data_manager) {
    data_managers_.push_back(data_manager);
  }

  void OnDataChannel(
      rtc::scoped_refptr<webrtc::DataChannelInterface> data_channel) override {
    for (RTCDataManager* data_manager : data_managers_) {
      data_manager->OnDataChannel(data_channel);
    }
  }

 private:
  std::vector<RTCDataManager*> data_managers_;
};

#endif