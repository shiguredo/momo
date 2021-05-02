#ifndef SORA_DATA_CHANNEL_H_
#define SORA_DATA_CHANNEL_H_

#include <map>
#include <memory>
#include <set>

// Boost
#include <boost/asio.hpp>

// WebRTC
#include <api/data_channel_interface.h>

#include "rtc/rtc_data_manager.h"

class SoraDataChannelObserver {
 public:
  ~SoraDataChannelObserver() {}
  virtual void OnStateChange(
      rtc::scoped_refptr<webrtc::DataChannelInterface> data_channel) = 0;
  virtual void OnMessage(
      rtc::scoped_refptr<webrtc::DataChannelInterface> data_channel,
      const webrtc::DataBuffer& buffer) = 0;
};

// 複数の DataChannel を纏めてコールバックで受け取るためのクラス
class SoraDataChannel : public RTCDataManager {
  struct Thunk : webrtc::DataChannelObserver,
                 std::enable_shared_from_this<Thunk> {
    SoraDataChannel* p;
    rtc::scoped_refptr<webrtc::DataChannelInterface> dc;
    void OnStateChange() override { p->OnStateChange(shared_from_this()); }
    void OnMessage(const webrtc::DataBuffer& buffer) override {
      p->OnMessage(shared_from_this(), buffer);
    }
    void OnBufferedAmountChange(uint64_t previous_amount) override {
      p->OnBufferedAmountChange(shared_from_this(), previous_amount);
    }
  };

 public:
  SoraDataChannel(SoraDataChannelObserver* observer) : observer_(observer) {}
  bool IsOpen(std::string label) const {
    return labels_.find(label) != labels_.end();
  }
  void Send(std::string label, const webrtc::DataBuffer& data) {
    auto it = labels_.find(label);
    if (it == labels_.end()) {
      return;
    }
    auto data_channel = it->second;
    data_channel->Send(data);
  }

 public:
  void OnDataChannel(
      rtc::scoped_refptr<webrtc::DataChannelInterface> data_channel) override {
    std::shared_ptr<Thunk> thunk(new Thunk());
    thunk->p = this;
    thunk->dc = data_channel;
    data_channel->RegisterObserver(thunk.get());
    thunks_.insert(std::make_pair(thunk, data_channel));
    labels_.insert(std::make_pair(data_channel->label(), data_channel));
  }

 private:
  void OnStateChange(std::shared_ptr<Thunk> thunk) {
    auto data_channel = thunks_.at(thunk);
    if (data_channel->state() == webrtc::DataChannelInterface::kClosed) {
      labels_.erase(data_channel->label());
      thunks_.erase(thunk);
    }
    observer_->OnStateChange(data_channel);
  }

  void OnMessage(std::shared_ptr<Thunk> thunk,
                 const webrtc::DataBuffer& buffer) {
    auto data_channel = thunks_.at(thunk);
    observer_->OnMessage(data_channel, buffer);
  }
  void OnBufferedAmountChange(std::shared_ptr<Thunk> thunk,
                              uint64_t previous_amount) {}

 private:
  std::map<std::shared_ptr<Thunk>,
           rtc::scoped_refptr<webrtc::DataChannelInterface>>
      thunks_;
  std::map<std::string, rtc::scoped_refptr<webrtc::DataChannelInterface>>
      labels_;
  SoraDataChannelObserver* observer_;
};

#endif