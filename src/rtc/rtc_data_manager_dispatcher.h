#ifndef RTC_DATA_MANAGER_DISPATCHER_H_
#define RTC_DATA_MANAGER_DISPATCHER_H_

#include <vector>

#include "rtc_data_manager.h"

class RTCDataManagerDispatcher : public RTCDataManager {
 public:
  void Add(std::shared_ptr<RTCDataManager> data_manager) {
    data_managers_.push_back(data_manager);
  }

  void OnDataChannel(webrtc::scoped_refptr<webrtc::DataChannelInterface>
                         data_channel) override {
    for (std::weak_ptr<RTCDataManager> wp : data_managers_) {
      auto data_manager = wp.lock();
      if (data_manager) {
        data_manager->OnDataChannel(data_channel);
      }
    }
    data_managers_.erase(
        std::remove_if(data_managers_.begin(), data_managers_.end(),
                       [](const std::weak_ptr<RTCDataManager>& wp) {
                         return wp.expired();
                       }),
        data_managers_.end());
  }

 private:
  std::vector<std::weak_ptr<RTCDataManager>> data_managers_;
};

#endif
