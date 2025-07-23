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
      webrtc::scoped_refptr<webrtc::DataChannelInterface> data_channel) = 0;
  virtual void OnMessage(
      webrtc::scoped_refptr<webrtc::DataChannelInterface> data_channel,
      const webrtc::DataBuffer& buffer) = 0;
};

// 複数の DataChannel を纏めてコールバックで受け取るためのクラス
class SoraDataChannel : public RTCDataManager {
  struct Thunk : webrtc::DataChannelObserver,
                 std::enable_shared_from_this<Thunk> {
    SoraDataChannel* p;
    webrtc::scoped_refptr<webrtc::DataChannelInterface> dc;
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
    if (!data.binary) {
      std::string str((const char*)data.data.cdata(),
                      (const char*)data.data.cdata() + data.size());
      RTC_LOG(LS_INFO) << "Send DataChannel label=" << label << " data=" << str;
    }
    auto data_channel = it->second;
    data_channel->Send(data);
  }
  void Close(const webrtc::DataBuffer& disconnect_message,
             std::function<void()> on_close) {
    auto it = labels_.find("signaling");
    if (it == labels_.end()) {
      on_close();
      return;
    }
    on_close_ = on_close;
    auto data_channel = it->second;
    data_channel->Send(disconnect_message);
  }

 public:
  void OnDataChannel(
      webrtc::scoped_refptr<webrtc::DataChannelInterface> data_channel) override {
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
      data_channel->UnregisterObserver();
      RTC_LOG(LS_INFO) << "DataChannel closed label=" << data_channel->label();
    }
    auto observer = observer_;
    auto on_close = on_close_;
    auto empty = thunks_.empty();
    if (on_close != nullptr && empty) {
      on_close_ = nullptr;
    }
    observer->OnStateChange(data_channel);
    // すべての Data Channel が閉じたら通知する
    if (on_close != nullptr && empty) {
      RTC_LOG(LS_INFO) << "DataChannel closed all";
      on_close();
    }
  }

  void OnMessage(std::shared_ptr<Thunk> thunk,
                 const webrtc::DataBuffer& buffer) {
    auto observer = observer_;
    auto data_channel = thunks_.at(thunk);
    observer->OnMessage(data_channel, buffer);
  }
  void OnBufferedAmountChange(std::shared_ptr<Thunk> thunk,
                              uint64_t previous_amount) {}

 private:
  std::map<std::shared_ptr<Thunk>,
           webrtc::scoped_refptr<webrtc::DataChannelInterface>>
      thunks_;
  std::map<std::string, webrtc::scoped_refptr<webrtc::DataChannelInterface>>
      labels_;
  SoraDataChannelObserver* observer_;
  std::function<void()> on_close_;
};

#endif